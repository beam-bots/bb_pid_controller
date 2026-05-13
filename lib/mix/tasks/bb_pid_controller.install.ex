# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbPidController.Install do
    @shortdoc "Installs BB.PID.Controller into a project"
    @moduledoc """
    #{@shortdoc}

    Adds a `BB.PID.Controller` to your robot module with the kp/ki/kd gains
    wired to runtime-tunable parameters under `:config.<name>.{kp,ki,kd}`. The
    setpoint/measurement/output topics are scaffolded as `TODO`s for you to
    fill in once you know the joint paths in your topology.

    ## Example

    ```bash
    mix igniter.install bb_pid_controller
    mix igniter.install bb_pid_controller --name shoulder_pid
    ```

    ## Options

    * `--robot` - The robot module (defaults to `{AppPrefix}.Robot`).
    * `--name` - The controller name and param group name (default `pid`).
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [robot: :string, name: :string],
        aliases: [r: :robot, n: :name]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options
      robot_module = BB.Igniter.robot_module(igniter)
      name = options |> Keyword.get(:name, "pid") |> String.to_atom()

      igniter
      |> Formatter.import_dep(:bb_pid_controller)
      |> BB.Igniter.add_controller(robot_module, name, controller_code(name))
      |> BB.Igniter.add_param_group(robot_module, [:config, name], param_group_body())
      |> Igniter.add_notice(todo_notice(name))
    end

    defp controller_code(name) do
      """
      controller :#{name}, {BB.PID.Controller,
        kp: param([:config, :#{name}, :kp]),
        ki: param([:config, :#{name}, :ki]),
        kd: param([:config, :#{name}, :kd]),
        output_min: -1.0,
        output_max: 1.0,
        setpoint_topic: [:TODO],
        setpoint_message: BB.Message.Actuator.Command.Position,
        setpoint_path: [:position],
        measurement_topic: [:TODO],
        measurement_message: BB.Message.Sensor.JointState,
        measurement_path: [:positions, 0],
        output_topic: [:TODO],
        output_message: BB.Message.Actuator.Command.Velocity,
        output_field: :velocity,
        output_frame_id: :TODO,
        rate: 100}
      """
    end

    defp param_group_body do
      """
      param :kp, type: :float, default: 1.0, doc: "Proportional gain"
      param :ki, type: :float, default: 0.0, doc: "Integral gain"
      param :kd, type: :float, default: 0.0, doc: "Derivative gain"
      """
    end

    defp todo_notice(name) do
      """
      bb_pid_controller: a :#{name} controller was scaffolded with kp/ki/kd as
      tunable parameters. Replace the `[:TODO]` topic paths and `:TODO`
      frame_id with values matching the joint you want to control.
      """
    end
  end
else
  defmodule Mix.Tasks.BbPidController.Install do
    @shortdoc "Installs BB.PID.Controller into a project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_pid_controller.install task requires igniter.

          mix igniter.install bb_pid_controller
      """)

      exit({:shutdown, 1})
    end
  end
end
