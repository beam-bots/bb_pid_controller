# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.PID.ControllerTest do
  use ExUnit.Case, async: true

  alias BB.Process, as: BBProcess

  defmodule TestRobot do
    @moduledoc false
    use BB

    controllers do
      controller(
        :test_pid,
        {BB.PID.Controller,
         kp: 1.0,
         ki: 0.1,
         kd: 0.01,
         output_min: -10.0,
         output_max: 10.0,
         setpoint_topic: [:actuator, :base_link, :joint1, :pid],
         setpoint_message: BB.Message.Actuator.Command.Position,
         setpoint_path: [:position],
         measurement_topic: [:sensor, :base_link, :joint1, :encoder],
         measurement_message: BB.Message.Sensor.JointState,
         measurement_path: [:positions, 0],
         output_topic: [:actuator, :base_link, :joint1, :servo],
         output_message: BB.Message.Actuator.Command.Velocity,
         output_field: :velocity,
         output_frame_id: :joint1,
         rate: 100}
      )
    end

    topology do
      link :base_link do
      end
    end
  end

  describe "controller starts and subscribes" do
    test "controller is started and registered" do
      start_supervised!(TestRobot)

      pid = BBProcess.whereis(TestRobot, :test_pid)
      assert is_pid(pid)
      assert Process.alive?(pid)
    end
  end

  describe "message processing" do
    test "controller publishes output when both setpoint and measurement are received" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, setpoint_msg} =
        BB.Message.new(
          BB.Message.Actuator.Command.Position,
          :joint1,
          position: 1.0
        )

      BB.PubSub.publish(TestRobot, [:actuator, :base_link, :joint1, :pid], setpoint_msg)

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :joint1,
          names: [:joint1],
          positions: [0.0]
        )

      BB.PubSub.publish(TestRobot, [:sensor, :base_link, :joint1, :encoder], measurement_msg)

      assert_receive {:bb, [:actuator, :base_link, :joint1, :servo],
                      %BB.Message{payload: payload}},
                     100

      assert %BB.Message.Actuator.Command.Velocity{velocity: velocity} = payload
      assert is_float(velocity)
      assert velocity > 0.0
    end

    test "controller filters messages by type" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, wrong_type_msg} =
        BB.Message.new(
          BB.Message.Actuator.Command.Velocity,
          :joint1,
          velocity: 1.0
        )

      BB.PubSub.publish(TestRobot, [:actuator, :base_link, :joint1, :pid], wrong_type_msg)

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :joint1,
          names: [:joint1],
          positions: [0.0]
        )

      BB.PubSub.publish(TestRobot, [:sensor, :base_link, :joint1, :encoder], measurement_msg)

      refute_receive {:bb, [:actuator, :base_link, :joint1, :servo], _}, 50
    end

    test "controller does not publish when only setpoint is received" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, setpoint_msg} =
        BB.Message.new(
          BB.Message.Actuator.Command.Position,
          :joint1,
          position: 1.0
        )

      BB.PubSub.publish(TestRobot, [:actuator, :base_link, :joint1, :pid], setpoint_msg)

      refute_receive {:bb, [:actuator, :base_link, :joint1, :servo], _}, 50
    end

    test "controller does not publish when only measurement is received" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :joint1,
          names: [:joint1],
          positions: [0.0]
        )

      BB.PubSub.publish(TestRobot, [:sensor, :base_link, :joint1, :encoder], measurement_msg)

      refute_receive {:bb, [:actuator, :base_link, :joint1, :servo], _}, 50
    end
  end

  describe "PID calculation" do
    test "proportional response to error" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, setpoint_msg} =
        BB.Message.new(
          BB.Message.Actuator.Command.Position,
          :joint1,
          position: 1.0
        )

      BB.PubSub.publish(TestRobot, [:actuator, :base_link, :joint1, :pid], setpoint_msg)

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :joint1,
          names: [:joint1],
          positions: [0.5]
        )

      BB.PubSub.publish(TestRobot, [:sensor, :base_link, :joint1, :encoder], measurement_msg)

      assert_receive {:bb, _, %BB.Message{payload: %{velocity: velocity}}}, 100
      assert velocity > 0, "velocity should be positive when measurement < setpoint"
    end

    test "output is clamped to limits" do
      start_supervised!(TestRobot)

      BB.PubSub.subscribe(TestRobot, [:actuator, :base_link, :joint1, :servo])

      {:ok, setpoint_msg} =
        BB.Message.new(
          BB.Message.Actuator.Command.Position,
          :joint1,
          position: 1000.0
        )

      BB.PubSub.publish(TestRobot, [:actuator, :base_link, :joint1, :pid], setpoint_msg)

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :joint1,
          names: [:joint1],
          positions: [0.0]
        )

      BB.PubSub.publish(TestRobot, [:sensor, :base_link, :joint1, :encoder], measurement_msg)

      assert_receive {:bb, _, %BB.Message{payload: %{velocity: velocity}}}, 100
      assert velocity <= 10.0, "velocity should be clamped to output_max"
    end
  end

  describe "validation" do
    defmodule DuplicateSourceRobot do
      @moduledoc false
      use BB

      controllers do
        controller(
          :bad_pid,
          {BB.PID.Controller,
           kp: 1.0,
           setpoint_topic: [:sensor, :joint1],
           setpoint_message: BB.Message.Sensor.JointState,
           setpoint_path: [:positions, 0],
           measurement_topic: [:sensor, :joint1],
           measurement_message: BB.Message.Sensor.JointState,
           measurement_path: [:positions, 1],
           output_topic: [:actuator, :joint1],
           output_message: BB.Message.Actuator.Command.Velocity,
           output_field: :velocity,
           output_frame_id: :joint1}
        )
      end

      topology do
        link :base_link do
        end
      end
    end

    test "rejects duplicate setpoint and measurement sources at runtime" do
      Process.flag(:trap_exit, true)

      assert {:error,
              {:shutdown,
               {:failed_to_start_child, _,
                {:shutdown, {:failed_to_start_child, :bad_pid, {:invalid_config, msg}}}}}} =
               DuplicateSourceRobot.start_link([])

      assert msg =~ "setpoint and measurement must have different"
    end

    defmodule EmptySetpointPathRobot do
      @moduledoc false
      use BB

      controllers do
        controller(
          :bad_pid,
          {BB.PID.Controller,
           kp: 1.0,
           setpoint_topic: [:actuator, :joint1],
           setpoint_message: BB.Message.Actuator.Command.Position,
           setpoint_path: [],
           measurement_topic: [:sensor, :joint1],
           measurement_message: BB.Message.Sensor.JointState,
           measurement_path: [:positions, 0],
           output_topic: [:actuator, :joint1, :out],
           output_message: BB.Message.Actuator.Command.Velocity,
           output_field: :velocity,
           output_frame_id: :joint1}
        )
      end

      topology do
        link :base_link do
        end
      end
    end

    test "rejects empty setpoint_path at runtime" do
      Process.flag(:trap_exit, true)

      assert {:error,
              {:shutdown,
               {:failed_to_start_child, _,
                {:shutdown, {:failed_to_start_child, :bad_pid, {:invalid_config, msg}}}}}} =
               EmptySetpointPathRobot.start_link([])

      assert msg =~ "setpoint_path cannot be empty"
    end

    defmodule EmptyMeasurementPathRobot do
      @moduledoc false
      use BB

      controllers do
        controller(
          :bad_pid,
          {BB.PID.Controller,
           kp: 1.0,
           setpoint_topic: [:actuator, :joint1],
           setpoint_message: BB.Message.Actuator.Command.Position,
           setpoint_path: [:position],
           measurement_topic: [:sensor, :joint1],
           measurement_message: BB.Message.Sensor.JointState,
           measurement_path: [],
           output_topic: [:actuator, :joint1, :out],
           output_message: BB.Message.Actuator.Command.Velocity,
           output_field: :velocity,
           output_frame_id: :joint1}
        )
      end

      topology do
        link :base_link do
        end
      end
    end

    test "rejects empty measurement_path at runtime" do
      Process.flag(:trap_exit, true)

      assert {:error,
              {:shutdown,
               {:failed_to_start_child, _,
                {:shutdown, {:failed_to_start_child, :bad_pid, {:invalid_config, msg}}}}}} =
               EmptyMeasurementPathRobot.start_link([])

      assert msg =~ "measurement_path cannot be empty"
    end

    defmodule BadOutputFieldRobot do
      @moduledoc false
      use BB

      controllers do
        controller(
          :bad_pid,
          {BB.PID.Controller,
           kp: 1.0,
           setpoint_topic: [:actuator, :joint1],
           setpoint_message: BB.Message.Actuator.Command.Position,
           setpoint_path: [:position],
           measurement_topic: [:sensor, :joint1],
           measurement_message: BB.Message.Sensor.JointState,
           measurement_path: [:positions, 0],
           output_topic: [:actuator, :joint1, :out],
           output_message: BB.Message.Actuator.Command.Velocity,
           output_field: :nonexistent,
           output_frame_id: :joint1}
        )
      end

      topology do
        link :base_link do
        end
      end
    end

    test "rejects non-existent output_field at runtime" do
      Process.flag(:trap_exit, true)

      assert {:error,
              {:shutdown,
               {:failed_to_start_child, _,
                {:shutdown, {:failed_to_start_child, :bad_pid, {:invalid_config, msg}}}}}} =
               BadOutputFieldRobot.start_link([])

      assert msg =~ "output_field :nonexistent not found"
    end
  end

  describe "path extraction with indexed access" do
    defmodule IndexedPathRobot do
      @moduledoc false
      use BB

      controllers do
        controller(
          :indexed_pid,
          {BB.PID.Controller,
           kp: 1.0,
           output_min: -10.0,
           output_max: 10.0,
           setpoint_topic: [:sensor, :setpoint],
           setpoint_message: BB.Message.Sensor.JointState,
           setpoint_path: [:positions, 1],
           measurement_topic: [:sensor, :measurement],
           measurement_message: BB.Message.Sensor.JointState,
           measurement_path: [:positions, 2],
           output_topic: [:actuator, :output],
           output_message: BB.Message.Actuator.Command.Velocity,
           output_field: :velocity,
           output_frame_id: :joint,
           rate: 100}
        )
      end

      topology do
        link :base_link do
        end
      end
    end

    test "extracts values at correct indices" do
      start_supervised!(IndexedPathRobot)

      BB.PubSub.subscribe(IndexedPathRobot, [:actuator, :output])

      {:ok, setpoint_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :arm,
          names: [:j0, :j1, :j2],
          positions: [0.0, 1.5, 0.0]
        )

      BB.PubSub.publish(IndexedPathRobot, [:sensor, :setpoint], setpoint_msg)

      {:ok, measurement_msg} =
        BB.Message.new(
          BB.Message.Sensor.JointState,
          :arm,
          names: [:j0, :j1, :j2],
          positions: [0.0, 0.0, 1.0]
        )

      BB.PubSub.publish(IndexedPathRobot, [:sensor, :measurement], measurement_msg)

      assert_receive {:bb, _, %BB.Message{payload: %{velocity: velocity}}}, 100
      assert velocity > 0.0, "error = 1.5 - 1.0 = 0.5, so velocity should be positive"
    end
  end
end
