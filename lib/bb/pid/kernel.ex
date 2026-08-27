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

  ## Two representations

  A kernel holds either BEAM floats or `Nx` tensors, and `new/1` picks by what
  it is handed: all-numeric gains give the scalar form, anything else the tensor
  form. `step/4` likewise runs the scalar law only when the kernel and all three
  arguments are plain numbers, and promotes to tensors otherwise. Nothing in the
  public API distinguishes them - `output/1` and `put_gains/2` take either, and
  `kernel.integral` reads through `Nx.to_number/1` in both cases.

  The split exists because a single loop pays the whole cost of `Nx` for none of
  the benefit. A `{}`-shaped `defn` step is roughly three orders of magnitude
  slower than the same arithmetic on floats, and allocates accordingly, which is
  the difference between closing a 200 Hz loop on a small target and not. The
  batch case keeps the `defn`, where vectorising over the batch axis is a real
  win and the per-call overhead is amortised across every loop in it.

  A scalar kernel is not usable inside a surrounding `defn` at `:f64` - its
  floats would be converted on entry at the `Nx` default of `:f32`. Call
  `to_tensors/1` first, or build the kernel with a tensor gain.

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
    gains = Map.new(@gains, fn gain -> {gain, fetch_gain(opts, gain)} end)

    if Enum.all?(gains, fn {_gain, value} -> is_number(value) end) do
      new_scalar(gains)
    else
      new_batched(gains)
    end
  end

  defp new_scalar(gains) do
    gains
    |> Map.new(fn {gain, value} -> {gain, value / 1} end)
    |> Map.merge(%{
      integral: 0.0,
      prev_measurement: 0.0,
      derivative: 0.0,
      output: 0.0,
      primed: 0.0
    })
    |> then(&struct!(__MODULE__, &1))
  end

  defp new_batched(gains) do
    gains = Map.new(gains, fn {gain, value} -> {gain, to_tensor(value)} end)

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
    if scalar_representation?(kernel) and
         Enum.all?(opts, fn {_gain, value} -> is_number(value) end) do
      put_each(kernel, opts, &(&1 / 1))
    else
      kernel = to_tensors(kernel)
      shape = Nx.shape(kernel.integral)

      put_each(kernel, opts, &Nx.broadcast(to_tensor(&1), shape))
    end
  end

  defp put_each(kernel, opts, coerce) do
    Enum.reduce(@gains, kernel, fn gain, kernel ->
      case Keyword.fetch(opts, gain) do
        {:ok, value} -> Map.put(kernel, gain, coerce.(value))
        :error -> kernel
      end
    end)
  end

  @doc """
  Whether this kernel holds BEAM floats rather than `Nx` tensors.

  A single loop built from numeric gains does; a batch, or anything built from a
  tensor, does not. Only of interest when handing a kernel to something that
  requires tensors - see `to_tensors/1`.
  """
  @spec scalar_representation?(t()) :: boolean()
  def scalar_representation?(%__MODULE__{kp: kp}), do: is_number(kp)

  @doc """
  Promote a scalar kernel to the tensor representation, leaving state intact.

  Only needed to hand a single-loop kernel to something that requires tensors -
  composing it into a surrounding `defn`, or growing it into a batch. `step/4`
  promotes on its own when it needs to.
  """
  @spec to_tensors(t()) :: t()
  def to_tensors(%__MODULE__{} = kernel) do
    if scalar_representation?(kernel) do
      Map.new(Map.from_struct(kernel), fn {field, value} -> {field, to_tensor(value)} end)
      |> then(&struct!(__MODULE__, &1))
    else
      kernel
    end
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
    if scalar_representation?(kernel) and is_number(setpoint) and is_number(measurement) and
         is_number(dt) do
      scalar_step(kernel, setpoint / 1, measurement / 1, dt / 1)
    else
      # Built as f64 tensors here rather than cast inside the defn: a plain float
      # argument is converted on entry at the default f32, and casting afterwards
      # preserves the rounding rather than undoing it. An integrator accumulating
      # f32 error over a long run is not worth the memory saved.
      do_step(to_tensors(kernel), to_tensor(setpoint), to_tensor(measurement), to_tensor(dt))
    end
  end

  # The same law as do_step/4 below, over BEAM floats rather than tensors. Every
  # line has a counterpart there and the two must not drift; the shared property
  # tests hold them to each other.
  defp scalar_step(kernel, setpoint, measurement, dt) do
    dt = max(dt, @min_dt)
    error = setpoint - measurement

    proportional = kernel.kp * error

    raw_derivative =
      -kernel.kd * (measurement - kernel.prev_measurement) / dt * kernel.primed

    derivative = kernel.derivative + kernel.tau * (raw_derivative - kernel.derivative)

    unsaturated_integral = kernel.integral + kernel.ki * error * dt
    unsaturated = proportional + unsaturated_integral + derivative

    output =
      unsaturated
      |> max(kernel.output_min)
      |> min(kernel.output_max)

    %{
      kernel
      | integral: unsaturated_integral + (output - unsaturated),
        prev_measurement: measurement,
        derivative: derivative,
        output: output,
        primed: 1.0
    }
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
