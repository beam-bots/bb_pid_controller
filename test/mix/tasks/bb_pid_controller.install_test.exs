# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbPidController.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "controller" do
    test "adds a PID controller with kp/ki/kd as param refs" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :pid,
      + |      {BB.PID.Controller,
      + |       kp: param([:config, :pid, :kp]),
      + |       ki: param([:config, :pid, :ki]),
      + |       kd: param([:config, :pid, :kd]),
      """)
    end

    test "scaffolds topic paths as TODOs" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |       setpoint_topic: [:TODO],
      """)
    end

    test "uses a custom controller name when --name is given" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install", ["--name", "shoulder_pid"])
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :shoulder_pid,
      """)
    end
  end

  describe "parameters group" do
    test "adds a :config.<name> param group with kp/ki/kd" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    group :config do
      + |      group :pid do
      + |        param(:kp, type: :float, default: 1.0, doc: "Proportional gain")
      """)
    end

    test "uses the custom name for the param group" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install", ["--name", "shoulder_pid"])
      |> assert_has_patch("lib/test/robot.ex", """
      + |      group :shoulder_pid do
      """)
    end
  end

  describe "formatter" do
    test "imports bb_pid_controller into .formatter.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_has_patch(".formatter.exs", """
      + |  import_deps: [:bb_pid_controller, :bb]
      """)
    end
  end

  describe "notice" do
    test "prints a TODO notice" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_has_notice(&String.contains?(&1, "TODO"))
    end
  end

  describe "idempotency" do
    test "running twice produces no further changes" do
      project_with_robot()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> apply_igniter!()
      |> Igniter.compose_task("bb_pid_controller.install")
      |> assert_unchanged()
    end
  end
end
