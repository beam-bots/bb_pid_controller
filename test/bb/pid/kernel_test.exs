# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.PID.KernelTest do
  use ExUnit.Case, async: true

  alias BB.PID.Kernel

  @state [:integral, :prev_measurement, :derivative, :output, :primed]

  defp scalar(tensor), do: Nx.to_number(tensor)
  defp list(tensor), do: Nx.to_flat_list(tensor)

  defp assert_agrees(scalar_kernel, tensor_kernel) do
    assert Kernel.scalar_representation?(scalar_kernel)
    refute Kernel.scalar_representation?(tensor_kernel)

    for field <- @state do
      assert_in_delta scalar(Map.fetch!(scalar_kernel, field)),
                      scalar(Map.fetch!(tensor_kernel, field)),
                      1.0e-12,
                      "#{field} diverged between the scalar and tensor kernels"
    end
  end

  defp drive(kernel, steps) do
    Enum.reduce(steps, kernel, fn {setpoint, measurement, dt}, kernel ->
      Kernel.step(kernel, setpoint, measurement, dt)
    end)
  end

  # A tensor gain forces the tensor representation for otherwise identical gains.
  defp pair(gains) do
    {Kernel.new(gains), Kernel.new(Keyword.put(gains, :kp, Nx.tensor(gains[:kp], type: :f64)))}
  end

  describe "new/1" do
    test "defaults every gain but kp" do
      kernel = Kernel.new(kp: 2.0)

      assert scalar(kernel.kp) == 2.0
      assert scalar(kernel.ki) == 0.0
      assert scalar(kernel.kd) == 0.0
      assert scalar(kernel.tau) == 1.0
      assert scalar(kernel.output_min) == -1.0
      assert scalar(kernel.output_max) == 1.0
    end

    test "zeroes loop state" do
      kernel = Kernel.new(kp: 1.0)

      assert scalar(kernel.integral) == 0.0
      assert scalar(kernel.prev_measurement) == 0.0
      assert scalar(kernel.derivative) == 0.0
      assert scalar(kernel.output) == 0.0
      assert scalar(kernel.primed) == 0.0
    end

    test "a scalar gain gives a scalar kernel" do
      assert Nx.shape(Kernel.new(kp: 1.0).integral) == {}
    end

    test "a list of gains gives a batched kernel, with state to match" do
      kernel = Kernel.new(kp: [1.0, 2.0, 3.0])

      assert Nx.shape(kernel.kp) == {3}
      assert Nx.shape(kernel.integral) == {3}
      assert list(kernel.ki) == [0.0, 0.0, 0.0]
    end

    test "a scalar gain broadcasts across a batch" do
      kernel = Kernel.new(kp: [1.0, 2.0], ki: 0.5)

      assert list(kernel.ki) == [0.5, 0.5]
    end
  end

  describe "proportional term" do
    test "is the gain times the error" do
      kernel = Kernel.new(kp: 2.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 1.0, 0.25, 0.01)

      assert_in_delta scalar(kernel.output), 1.5, 1.0e-9
    end

    test "reverses sign with the error" do
      kernel = Kernel.new(kp: 2.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 0.0, 1.0, 0.01)

      assert_in_delta scalar(kernel.output), -2.0, 1.0e-9
    end
  end

  describe "integral term" do
    test "accumulates error scaled by dt, not by step count" do
      # ki=1, e=1 held for one second, reached two ways.
      one_step = Kernel.step(Kernel.new(kp: 0.0, ki: 1.0, output_max: 100.0), 1.0, 0.0, 1.0)

      ten_steps =
        Enum.reduce(1..10, Kernel.new(kp: 0.0, ki: 1.0, output_max: 100.0), fn _, kernel ->
          Kernel.step(kernel, 1.0, 0.0, 0.1)
        end)

      assert_in_delta scalar(one_step.integral), 1.0, 1.0e-9
      assert_in_delta scalar(ten_steps.integral), 1.0, 1.0e-9
    end

    test "is unaffected by kd" do
      kernel = Kernel.new(kp: 0.0, ki: 1.0, kd: 5.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 1.0, 0.0, 0.5)

      assert_in_delta scalar(kernel.integral), 0.5, 1.0e-9
    end
  end

  describe "derivative term" do
    test "is zero on the first step, having nothing to differentiate" do
      kernel = Kernel.new(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 0.0, 5.0, 0.1)

      assert scalar(kernel.derivative) == 0.0
      assert scalar(kernel.output) == 0.0
    end

    test "opposes a rising measurement" do
      kernel = Kernel.new(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 0.0, 0.0, 0.1)
      kernel = Kernel.step(kernel, 0.0, 1.0, 0.1)

      # -kd * Δpv / dt = -1 * 1.0 / 0.1
      assert_in_delta scalar(kernel.output), -10.0, 1.0e-9
    end

    test "ignores a setpoint step - no derivative kick" do
      # The defining reason for differentiating the measurement instead of the
      # error: with derivative-on-error this step would spike by kd * Δsp / dt.
      kernel = Kernel.new(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 0.0, 0.0, 0.1)
      kernel = Kernel.step(kernel, 50.0, 0.0, 0.1)

      assert scalar(kernel.output) == 0.0
    end

    test "tau attenuates the filtered term" do
      unfiltered =
        Kernel.new(kp: 0.0, kd: 1.0, tau: 1.0, output_min: -100.0, output_max: 100.0)

      filtered =
        Kernel.new(kp: 0.0, kd: 1.0, tau: 0.25, output_min: -100.0, output_max: 100.0)

      step = fn kernel ->
        kernel
        |> Kernel.step(0.0, 0.0, 0.1)
        |> Kernel.step(0.0, 1.0, 0.1)
      end

      assert_in_delta scalar(step.(unfiltered).output), -10.0, 1.0e-9
      assert_in_delta scalar(step.(filtered).output), -2.5, 1.0e-9
    end

    test "a spuriously small dt cannot produce an unbounded spike" do
      kernel = Kernel.new(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)
      kernel = Kernel.step(kernel, 0.0, 0.0, 0.1)
      kernel = Kernel.step(kernel, 0.0, 1.0, 0.0)

      # Floored at @min_dt, so the term is merely large and clamps, rather than
      # dividing by zero and poisoning the integrator with an infinity.
      assert scalar(kernel.output) == -100.0
      assert Nx.to_number(Nx.is_infinity(kernel.integral)) == 0
    end
  end

  describe "output clamping" do
    test "clamps to output_max" do
      kernel = Kernel.step(Kernel.new(kp: 100.0, output_max: 1.0), 1.0, 0.0, 0.01)

      assert scalar(kernel.output) == 1.0
    end

    test "clamps to output_min" do
      kernel = Kernel.step(Kernel.new(kp: 100.0, output_min: -1.0), 0.0, 1.0, 0.01)

      assert scalar(kernel.output) == -1.0
    end
  end

  describe "anti-windup" do
    test "holds p + i + d equal to the clamped output while saturated" do
      # The defining property of back-calculation. pid_control instead clamped
      # the integrator to the output range, which bounds the wrong quantity.
      kernel = Kernel.new(kp: 10.0, ki: 1.0, output_max: 1.0)
      kernel = Kernel.step(kernel, 1.0, 0.0, 1.0)

      terms =
        scalar(Nx.multiply(kernel.kp, 1.0)) + scalar(kernel.integral) +
          scalar(kernel.derivative)

      assert scalar(kernel.output) == 1.0
      assert_in_delta terms, 1.0, 1.0e-9
    end

    test "the integrator stops growing once the output saturates" do
      kernel = Kernel.new(kp: 10.0, ki: 1.0, output_max: 1.0)

      after_one = Kernel.step(kernel, 1.0, 0.0, 1.0)
      after_many = Enum.reduce(1..20, kernel, fn _, k -> Kernel.step(k, 1.0, 0.0, 1.0) end)

      assert_in_delta scalar(after_one.integral), scalar(after_many.integral), 1.0e-9
    end

    test "the output responds on the first step after the error reverses" do
      # A wound-up integrator clamped to the output range would spend several
      # steps unwinding before the output moved at all.
      kernel = Kernel.new(kp: 10.0, ki: 1.0, output_min: -1.0, output_max: 1.0)
      saturated = Enum.reduce(1..20, kernel, fn _, k -> Kernel.step(k, 1.0, 0.0, 1.0) end)
      assert scalar(saturated.output) == 1.0

      reversed = Kernel.step(saturated, 0.0, 1.0, 1.0)

      assert scalar(reversed.output) < 0.0
    end
  end

  describe "batching" do
    test "advances independent loops with independent gains in one call" do
      kernel = Kernel.new(kp: [1.0, 10.0], output_max: 100.0)
      kernel = Kernel.step(kernel, [1.0, 1.0], [0.0, 0.0], 0.01)

      assert list(kernel.output) == [1.0, 10.0]
    end

    test "each loop carries its own integrator" do
      kernel = Kernel.new(kp: 0.0, ki: [1.0, 2.0], output_max: 100.0)
      kernel = Kernel.step(kernel, [1.0, 1.0], [0.0, 0.0], 1.0)

      assert list(kernel.integral) == [1.0, 2.0]
    end

    test "a batch matches the same loops stepped one at a time" do
      inputs = [{2.0, 0.5, 1.0, 0.3}, {0.5, 0.1, 0.2, 0.9}]

      batched =
        Kernel.new(
          kp: Enum.map(inputs, &elem(&1, 0)),
          ki: Enum.map(inputs, &elem(&1, 1)),
          kd: Enum.map(inputs, &elem(&1, 2)),
          output_min: -100.0,
          output_max: 100.0
        )

      setpoints = Enum.map(inputs, &elem(&1, 3))

      batched =
        Enum.reduce([0.0, 0.1, 0.25], batched, fn measurement, kernel ->
          Kernel.step(kernel, setpoints, [measurement, measurement], 0.02)
        end)

      individual =
        for {kp, ki, kd, setpoint} <- inputs do
          Enum.reduce(
            [0.0, 0.1, 0.25],
            Kernel.new(
              kp: kp,
              ki: ki,
              kd: kd,
              output_min: -100.0,
              output_max: 100.0
            ),
            fn measurement, kernel -> Kernel.step(kernel, setpoint, measurement, 0.02) end
          )
          |> Kernel.output()
        end

      assert_in_delta Enum.at(list(batched.output), 0), Enum.at(individual, 0), 1.0e-9
      assert_in_delta Enum.at(list(batched.output), 1), Enum.at(individual, 1), 1.0e-9
    end

    test "loops prime independently" do
      kernel = Kernel.new(kp: [1.0, 1.0])
      assert list(kernel.primed) == [0.0, 0.0]

      kernel = Kernel.step(kernel, [0.0, 0.0], [0.0, 0.0], 0.1)
      assert list(kernel.primed) == [1.0, 1.0]
    end
  end

  describe "precision" do
    test "numbers and lists are taken as f64, not the Nx f32 default" do
      # Exact equality, not a delta: neither 0.3 nor 0.9 survives f32.
      kernel = Kernel.new(kp: 0.0, ki: [1.0, 1.0], output_max: 100.0)
      kernel = Kernel.step(kernel, [0.3, 0.9], [0.0, 0.0], 0.1)

      assert Nx.type(kernel.integral) == {:f, 64}
      assert list(kernel.integral) == [0.3 * 0.1, 0.9 * 0.1]
    end

    test "a tensor the caller built at f32 keeps its rounding" do
      # Documented, not fixable here: the precision went at Nx.tensor/1.
      kernel = Kernel.new(kp: 0.0, ki: 1.0, output_max: 100.0)
      kernel = Kernel.step(kernel, Nx.tensor(0.3), 0.0, 0.1)

      assert Nx.type(kernel.integral) == {:f, 64}
      refute scalar(kernel.integral) == 0.3 * 0.1
      assert_in_delta scalar(kernel.integral), 0.3 * 0.1, 1.0e-8
    end
  end

  describe "put_gains/2" do
    test "replaces only the named gains" do
      kernel = Kernel.new(kp: 1.0, ki: 2.0, kd: 3.0)
      retuned = Kernel.put_gains(kernel, kp: 9.0)

      assert scalar(retuned.kp) == 9.0
      assert scalar(retuned.ki) == 2.0
      assert scalar(retuned.kd) == 3.0
    end

    test "keeps accumulated loop state" do
      kernel = Kernel.new(kp: 0.0, ki: 1.0, output_max: 100.0)
      wound = Kernel.step(kernel, 1.0, 0.25, 0.5)
      retuned = Kernel.put_gains(wound, kp: 5.0)

      assert scalar(retuned.integral) == scalar(wound.integral)
      assert scalar(retuned.prev_measurement) == 0.25
      assert scalar(retuned.primed) == 1.0
    end

    test "keeps the kernel steppable after a retune" do
      kernel = Kernel.new(kp: 1.0, output_max: 100.0)
      retuned = Kernel.put_gains(kernel, kp: 3.0)
      stepped = Kernel.step(retuned, 1.0, 0.0, 0.01)

      assert_in_delta scalar(stepped.output), 3.0, 1.0e-9
    end
  end

  describe "the scalar and tensor representations agree" do
    # The two write the same control law twice, once over floats and once over
    # tensors. Everything here drives both with identical inputs and holds their
    # whole state to each other, so a change to one that is not made to the
    # other fails rather than silently changing the single-loop behaviour.

    test "over a full PID with every term active" do
      {s, t} = pair(kp: 2.0, ki: 0.5, kd: 0.1, tau: 0.8, output_min: -50.0, output_max: 50.0)

      steps = [{1.0, 0.0, 0.01}, {1.0, 0.3, 0.01}, {1.0, 0.7, 0.01}, {0.0, 0.9, 0.02}]

      assert_agrees(drive(s, steps), drive(t, steps))
    end

    test "through saturation and back-calculation anti-windup" do
      {s, t} = pair(kp: 10.0, ki: 1.0, output_max: 1.0)

      steps = List.duplicate({1.0, 0.0, 1.0}, 5) ++ List.duplicate({-1.0, 0.0, 1.0}, 5)

      assert_agrees(drive(s, steps), drive(t, steps))
    end

    test "on the first step, where the derivative is held at zero" do
      {s, t} = pair(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)

      assert_agrees(drive(s, [{0.0, 5.0, 0.1}]), drive(t, [{0.0, 5.0, 0.1}]))
    end

    test "on a spuriously small dt, where both floor at @min_dt" do
      {s, t} = pair(kp: 0.0, kd: 1.0, output_min: -100.0, output_max: 100.0)

      steps = [{0.0, 0.0, 0.1}, {0.0, 1.0, 0.0}]

      assert_agrees(drive(s, steps), drive(t, steps))
    end

    test "after a retune that keeps accumulated state" do
      {s, t} = pair(kp: 0.0, ki: 1.0, output_max: 100.0)

      retune = fn kernel ->
        kernel
        |> drive([{1.0, 0.25, 0.5}])
        |> Kernel.put_gains(kp: 5.0, ki: 0.25)
        |> drive([{1.0, 0.5, 0.5}])
      end

      assert_agrees(retune.(s), retune.(t))
    end
  end

  describe "promotion between representations" do
    test "new/1 gives the scalar representation for all-numeric gains" do
      assert Kernel.scalar_representation?(Kernel.new(kp: 2.0, ki: 0.5))
    end

    test "new/1 gives the tensor representation for a tensor gain" do
      refute Kernel.scalar_representation?(Kernel.new(kp: Nx.tensor(2.0)))
    end

    test "new/1 gives the tensor representation for a batch" do
      refute Kernel.scalar_representation?(Kernel.new(kp: [1.0, 2.0]))
    end

    test "step/4 promotes a scalar kernel given a tensor argument" do
      stepped = Kernel.step(Kernel.new(kp: 1.0), Nx.tensor(1.0, type: :f64), 0.0, 0.01)

      refute Kernel.scalar_representation?(stepped)
      assert_in_delta scalar(stepped.output), 1.0, 1.0e-9
    end

    test "step/4 promotes a scalar kernel given a batched argument" do
      stepped = Kernel.step(Kernel.new(kp: 1.0), [1.0, 2.0], 0.0, 0.01)

      refute Kernel.scalar_representation?(stepped)
      assert list(stepped.output) == [1.0, 1.0]
    end

    test "put_gains/2 promotes a scalar kernel given a tensor gain" do
      retuned = Kernel.put_gains(Kernel.new(kp: 1.0), kp: Nx.tensor(2.0, type: :f64))

      refute Kernel.scalar_representation?(retuned)
      assert scalar(retuned.kp) == 2.0
    end

    test "put_gains/2 cannot grow a single loop into a batch" do
      # Not a limitation of the scalar representation - the tensor kernel has
      # always refused this, because the accumulated state has no batch axis to
      # broadcast the new gain against.
      assert_raise ArgumentError, ~r/cannot broadcast/, fn ->
        Kernel.put_gains(Kernel.new(kp: 1.0), kp: [2.0, 3.0])
      end
    end

    test "to_tensors/1 preserves state and is idempotent" do
      wound = Kernel.step(Kernel.new(kp: 1.0, ki: 1.0, output_max: 100.0), 1.0, 0.25, 0.5)
      promoted = Kernel.to_tensors(wound)

      refute Kernel.scalar_representation?(promoted)
      assert Nx.type(promoted.integral) == {:f, 64}
      assert scalar(promoted.integral) == scalar(wound.integral)
      assert scalar(promoted.prev_measurement) == 0.25
      assert Kernel.to_tensors(promoted) == promoted
    end
  end
end
