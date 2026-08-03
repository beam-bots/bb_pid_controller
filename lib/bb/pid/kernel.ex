# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.PID.Kernel do
  @moduledoc """
  The PID control law as a composable `Nx.Defn` kernel.

  `step/4` is written elementwise over its tensors, so the same kernel drives one
  loop or a thousand. Gains and state are both tensors of the same shape: `{}`
  for a single loop, `{n}` for `n` loops advanced in one call, and any shape
  beyond that if you have a reason. Nothing here reshapes or branches per loop.

  This is why the law lives in a `defn` rather than in the GenServer that
  usually calls it - it can be JIT-compiled, vectorised over a batch axis, and
  composed into a larger computation, none of which is possible for a control
  law locked behind a process. `BB.PID.Controller` is just the `{}` case.

  ## Usage

      kernel = BB.PID.Kernel.new(kp: 2.0, ki: 0.5, kd: 0.1)
      kernel = BB.PID.Kernel.step(kernel, _setpoint = 1.0, _measurement = 0.0, _dt = 0.01)
      BB.PID.Kernel.output(kernel)

  Batched, with per-loop gains:

      kernel = BB.PID.Kernel.new(kp: [2.0, 1.0], ki: [0.5, 0.0], kd: [0.0, 0.1])

      kernel =
        BB.PID.Kernel.step(
          kernel,
          Nx.tensor([1.0, 0.5]),
          Nx.tensor([0.0, 0.4]),
          0.01
        )

  ## The control law

  Two deliberate departures from the textbook form, both of which the
  `pid_control` package this replaced got wrong.

  **The derivative acts on the measurement, not the error.** With `e = sp - pv`,
  differentiating the error means a step change in setpoint puts a spike of
  `kd * Δsp / dt` through the output - "derivative kick". Differentiating `pv`
  and negating gives the same response to disturbances with no sensitivity to
  setpoint changes at all.

  **Anti-windup is by back-calculation.** When the output saturates, the
  saturation error is folded straight back into the integrator:

      integral = unsaturated_integral + (output - unsaturated_output)

  which holds `p + i + d == output` exactly while clamped, so the integrator
  stops accumulating the instant the output hits a limit and starts moving again
  the instant the error reverses. Clamping the integrator to the *output* range
  instead - as `pid_control` did - bounds the wrong quantity: with a large `kp`
  the proportional term alone can fill the range, leaving the integrator free to
  wind up to a limit it then has to unwind before the output responds at all.

  ## Precision

  Everything the kernel builds is `:f64`, and numbers and lists handed to
  `step/4` are converted to `:f64` too - so `step(kernel, [0.3, 0.9], ...)` is
  exact where `Nx.tensor([0.3, 0.9])` would not be. This has to happen at the
  boundary: `defn` converts a plain float argument to a tensor at the `Nx`
  default of `:f32` on entry, and casting inside the `defn` preserves that
  rounding rather than undoing it. Hence the `deftransform`, which also keeps
  `step/4` callable from within another `defn`.

  The one lossy path left is handing in a tensor you built yourself at `:f32`.
  It is upcast, but the precision went when the tensor was created. Build batched
  inputs as lists, or as `Nx.tensor(values, type: :f64)`.

  ## Gains are per second

  `ki` and `kd` are applied against the measured `dt` handed to `step/4`, so
  they are gains per second of accumulated error and per second of measurement
  change respectively. They do not depend on the loop's rate.

  ## The first step

  A loop has no previous measurement to differentiate on its first step, so the
  derivative term is held at zero until one has been recorded. The `:primed`
  field tracks this per loop, which matters under batching - loops added to a
  batch at different times each prime on their own first step.
  """

  import Nx.Defn

  @derive {Nx.Container,
           containers: [
             :kp,
             :ki,
             :kd,
             :tau,
             :output_min,
             :output_max,
             :integral,
             :prev_measurement,
             :derivative,
             :output,
             :primed
           ]}

  defstruct [
    :kp,
    :ki,
    :kd,
    :tau,
    :output_min,
    :output_max,
    :integral,
    :prev_measurement,
    :derivative,
    :output,
    :primed
  ]

  @type t :: %__MODULE__{}

  # A dt small enough to be spurious rather than real. Dividing the measurement
  # delta by it would put an arbitrarily large spike through the derivative and
  # poison the integrator, which is precisely the failure this module's timing
  # discipline exists to avoid.
  @min_dt 1.0e-6

  @gains [:kp, :ki, :kd, :tau, :output_min, :output_max]

  @doc """
  Build a kernel from gains.

  Each gain accepts a number for a single loop, or a list (or tensor) to drive a
  batch. All gains must agree on shape. State fields are zeroed to match.

  ## Options

  - `:kp` (required) - proportional gain.
  - `:ki` - integral gain per second. Default `0.0`.
  - `:kd` - derivative gain per second. Default `0.0`.
  - `:tau` - low-pass coefficient applied to the derivative term, in `0..1`.
    `1.0` (the default) is unfiltered; lower values attenuate measurement noise
    at the cost of lag.
  - `:output_min` / `:output_max` - output clamp. Defaults `-1.0` / `1.0`.
  """
  @spec new(keyword()) :: t()
  def new(opts) do
    gains = Map.new(@gains, fn gain -> {gain, to_tensor(fetch_gain(opts, gain))} end)

    shape = batch_shape(Map.values(gains))
    zeros = Nx.broadcast(to_tensor(0.0), shape)

    gains
    |> Map.new(fn {gain, value} -> {gain, Nx.broadcast(value, shape)} end)
    |> Map.merge(%{
      integral: zeros,
      prev_measurement: zeros,
      derivative: zeros,
      output: zeros,
      primed: zeros
    })
    |> then(&struct!(__MODULE__, &1))
  end

  # Gains may be given per loop or as a scalar to share across the batch, so the
  # kernel's shape is the one they broadcast to rather than any single gain's.
  defp batch_shape(tensors) do
    tensors
    |> Enum.map(&Nx.shape/1)
    |> Enum.uniq()
    |> Enum.reject(&(&1 == {}))
    |> case do
      [] -> {}
      [shape] -> shape
      shapes -> raise ArgumentError, "gains must agree on shape, got #{inspect(shapes)}"
    end
  end

  defp fetch_gain(opts, gain) do
    case Keyword.fetch(opts, gain) do
      {:ok, value} -> value
      :error -> default(gain)
    end
  end

  defp default(:kp), do: raise(ArgumentError, "the :kp gain is required")
  defp default(:tau), do: 1.0
  defp default(:output_min), do: -1.0
  defp default(:output_max), do: 1.0
  defp default(_gain), do: 0.0

  @doc """
  Replace the gains, leaving accumulated loop state intact.

  Retuning mid-flight must not discard the integrator: dropping it steps the
  output by whatever the integral had accumulated, which on a loaded joint is a
  visible jolt. Only the gains named in `opts` change.
  """
  @spec put_gains(t(), keyword()) :: t()
  def put_gains(%__MODULE__{} = kernel, opts) do
    shape = Nx.shape(kernel.integral)

    Enum.reduce(@gains, kernel, fn gain, kernel ->
      case Keyword.fetch(opts, gain) do
        {:ok, value} -> Map.put(kernel, gain, Nx.broadcast(to_tensor(value), shape))
        :error -> kernel
      end
    end)
  end

  @doc """
  The most recent output as a number, for a single-loop kernel.

  Batched kernels should read `kernel.output` as a tensor rather than forcing it
  through a scalar; `Nx.to_number/1` on a non-scalar raises.
  """
  @spec output(t()) :: number()
  def output(%__MODULE__{output: output}), do: Nx.to_number(output)

  @doc """
  Advance the loop by one step over the elapsed time `dt`, in seconds.

  `setpoint`, `measurement` and `dt` each accept a number, a list, or a tensor,
  and broadcast against the kernel's shape - so a batch may share a setpoint or a
  `dt` while carrying its own measurements. Numbers and lists are converted to
  `:f64`; see the precision note above.
  """
  deftransform step(kernel, setpoint, measurement, dt) do
    # Built as f64 tensors here rather than cast inside the defn: a plain float
    # argument is converted on entry at the default f32, and casting afterwards
    # preserves the rounding rather than undoing it. An integrator accumulating
    # f32 error over a long run is not worth the memory saved.
    do_step(kernel, to_tensor(setpoint), to_tensor(measurement), to_tensor(dt))
  end

  defnp do_step(kernel, setpoint, measurement, dt) do
    dt = Nx.max(dt, @min_dt)
    error = setpoint - measurement

    proportional = kernel.kp * error

    # On the measurement rather than the error, and zeroed until the loop has a
    # previous measurement to differentiate against.
    raw_derivative =
      -kernel.kd * (measurement - kernel.prev_measurement) / dt * kernel.primed

    derivative = kernel.derivative + kernel.tau * (raw_derivative - kernel.derivative)

    unsaturated_integral = kernel.integral + kernel.ki * error * dt
    unsaturated = proportional + unsaturated_integral + derivative
    # Not Nx.clip/3, which takes scalar bounds only and so cannot express a
    # per-loop output range across a batch.
    output =
      unsaturated
      |> Nx.max(kernel.output_min)
      |> Nx.min(kernel.output_max)

    %{
      kernel
      | integral: unsaturated_integral + (output - unsaturated),
        prev_measurement: measurement,
        derivative: derivative,
        output: output,
        primed: Nx.as_type(Nx.broadcast(1.0, kernel.primed), :f64)
    }
  end

  defp to_tensor(%Nx.Tensor{} = tensor), do: Nx.as_type(tensor, :f64)
  defp to_tensor(value), do: Nx.tensor(value, type: :f64)
end
