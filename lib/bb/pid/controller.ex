# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.PID.Controller do
  @moduledoc """
  General-purpose PID controller for BB robots.

  Subscribes to configurable topics for setpoint and measurement values,
  runs a periodic PID loop, and publishes output to a configurable topic.

  ## Architecture

  ```
  Setpoint Topic ─────────────────────────┐
  (configurable message/field)            │
                                          ▼
                                 ┌─────────────────┐
  Measurement Topic ────────────►│ BB.PID.Controller│
  (configurable message/field)   │                  │
                                 │   PIDControl     │
                                 │   .step()        │
                                 └────────┬─────────┘
                                          │
                                          ▼
                                 Output Topic
                                 (configurable message/field)
  ```

  One controller instance = one PID loop. Instantiate multiple controllers
  for multiple control loops.

  ## Example Usage

      defmodule MyRobot do
        use BB

        controller :shoulder_pid, {BB.PID.Controller,
          kp: 2.0, ki: 0.1, kd: 0.05,
          output_min: -1.0, output_max: 1.0,

          setpoint_topic: [:actuator, :base_link, :shoulder, :pid],
          setpoint_message: BB.Message.Actuator.Command.Position,
          setpoint_path: [:position],

          measurement_topic: [:sensor, :base_link, :shoulder, :encoder],
          measurement_message: BB.Message.Sensor.JointState,
          measurement_path: [:positions, 0],

          output_topic: [:actuator, :base_link, :shoulder, :servo],
          output_message: BB.Message.Actuator.Command.Velocity,
          output_field: :velocity,
          output_frame_id: :shoulder,

          rate: 100
        }

        # ... rest of robot definition
      end

  ## Path Extraction

  The `*_path` options are lists of atoms (field names) and integers (list indices):

      setpoint_path: [:position]          # extracts payload.position
      measurement_path: [:positions, 0]   # extracts payload.positions |> Enum.at(0)
      path: [:data, :readings, 0, :value] # extracts payload.data.readings[0].value
  """

  use BB.Controller,
    options_schema: [
      kp: [type: :float, required: true, doc: "Proportional gain"],
      ki: [type: :float, default: 0.0, doc: "Integral gain"],
      kd: [type: :float, default: 0.0, doc: "Derivative gain"],
      tau: [type: :float, default: 1.0, doc: "Derivative low-pass filter (0-1, 1=no filter)"],
      output_min: [type: :float, default: -1.0, doc: "Output clamp minimum"],
      output_max: [type: :float, default: 1.0, doc: "Output clamp maximum"],
      setpoint_topic: [
        type: {:list, :atom},
        required: true,
        doc: "Topic path to subscribe to for setpoint"
      ],
      setpoint_message: [
        type: {:behaviour, BB.Message},
        required: true,
        doc: "Message module to filter for (e.g., BB.Message.Actuator.Command.Position)"
      ],
      setpoint_path: [
        type: {:list, {:or, [:atom, :non_neg_integer]}},
        required: true,
        doc: "Path to value in message payload (e.g., [:position] or [:positions, 0])"
      ],
      measurement_topic: [
        type: {:list, :atom},
        required: true,
        doc: "Topic path to subscribe to for measurement"
      ],
      measurement_message: [
        type: {:behaviour, BB.Message},
        required: true,
        doc: "Message module to filter for (e.g., BB.Message.Sensor.JointState)"
      ],
      measurement_path: [
        type: {:list, {:or, [:atom, :non_neg_integer]}},
        required: true,
        doc: "Path to value in message payload (e.g., [:positions, 0])"
      ],
      output_topic: [
        type: {:list, :atom},
        required: true,
        doc: "Topic path to publish output to"
      ],
      output_message: [
        type: {:behaviour, BB.Message},
        required: true,
        doc: "Message module to construct (e.g., BB.Message.Actuator.Command.Velocity)"
      ],
      output_field: [
        type: :atom,
        required: true,
        doc: "Field name in output message for PID output value (e.g., :velocity)"
      ],
      output_frame_id: [type: :atom, required: true, doc: "frame_id for constructed messages"],
      rate: [type: :pos_integer, default: 100, doc: "Control loop frequency (Hz)"]
    ]

  @impl BB.Controller
  def init(opts) do
    with :ok <- validate_unique_sources(opts),
         :ok <- validate_paths(opts),
         :ok <- validate_output_field(opts) do
      bb = Keyword.fetch!(opts, :bb)

      pid =
        PIDControl.new(
          kp: Keyword.fetch!(opts, :kp),
          ki: Keyword.get(opts, :ki, 0.0),
          kd: Keyword.get(opts, :kd, 0.0),
          tau: Keyword.get(opts, :tau, 1.0),
          output_min: Keyword.get(opts, :output_min, -1.0),
          output_max: Keyword.get(opts, :output_max, 1.0)
        )

      state = %{
        bb: bb,
        pid: pid,
        setpoint: nil,
        measurement: nil,
        setpoint_topic: opts[:setpoint_topic],
        setpoint_message: opts[:setpoint_message],
        setpoint_path: opts[:setpoint_path],
        measurement_topic: opts[:measurement_topic],
        measurement_message: opts[:measurement_message],
        measurement_path: opts[:measurement_path],
        output_topic: opts[:output_topic],
        output_message: opts[:output_message],
        output_field: opts[:output_field],
        output_frame_id: opts[:output_frame_id],
        rate: opts[:rate],
        tick_ref: nil
      }

      BB.PubSub.subscribe(bb.robot, state.setpoint_topic)
      BB.PubSub.subscribe(bb.robot, state.measurement_topic)

      tick_ref = schedule_tick(state.rate)
      {:ok, %{state | tick_ref: tick_ref}}
    end
  end

  @impl BB.Controller
  def handle_info({:bb, topic, %BB.Message{payload: %type{} = payload}}, state) do
    cond do
      topic == state.setpoint_topic and type == state.setpoint_message ->
        value = extract_value(payload, state.setpoint_path)
        {:noreply, %{state | setpoint: value}}

      topic == state.measurement_topic and type == state.measurement_message ->
        value = extract_value(payload, state.measurement_path)
        {:noreply, %{state | measurement: value}}

      true ->
        {:noreply, state}
    end
  end

  def handle_info(:tick, state) do
    state =
      if state.setpoint != nil and state.measurement != nil do
        pid = PIDControl.step(state.pid, state.setpoint, state.measurement)
        message = build_output_message(pid.output, state)
        BB.PubSub.publish(state.bb.robot, state.output_topic, message)
        %{state | pid: pid}
      else
        state
      end

    tick_ref = schedule_tick(state.rate)
    {:noreply, %{state | tick_ref: tick_ref}}
  end

  def handle_info(_msg, state) do
    {:noreply, state}
  end

  @impl BB.Controller
  def handle_options(new_opts, state) do
    pid =
      PIDControl.new(
        kp: Keyword.fetch!(new_opts, :kp),
        ki: Keyword.get(new_opts, :ki, 0.0),
        kd: Keyword.get(new_opts, :kd, 0.0),
        tau: Keyword.get(new_opts, :tau, 1.0),
        output_min: Keyword.get(new_opts, :output_min, -1.0),
        output_max: Keyword.get(new_opts, :output_max, 1.0)
      )

    {:ok, %{state | pid: pid}}
  end

  @impl BB.Controller
  def terminate(_reason, state) do
    if state.tick_ref, do: Process.cancel_timer(state.tick_ref)
    :ok
  end

  defp validate_unique_sources(opts) do
    setpoint_source = {opts[:setpoint_topic], opts[:setpoint_message]}
    measurement_source = {opts[:measurement_topic], opts[:measurement_message]}

    if setpoint_source == measurement_source do
      {:stop,
       {:invalid_config,
        "setpoint and measurement must have different topic/message combinations"}}
    else
      :ok
    end
  end

  defp validate_paths(opts) do
    cond do
      opts[:setpoint_path] == [] ->
        {:stop, {:invalid_config, "setpoint_path cannot be empty"}}

      opts[:measurement_path] == [] ->
        {:stop, {:invalid_config, "measurement_path cannot be empty"}}

      true ->
        :ok
    end
  end

  defp validate_output_field(opts) do
    output_message = opts[:output_message]
    output_field = opts[:output_field]
    schema = output_message.schema()

    case Keyword.fetch(schema.schema, output_field) do
      {:ok, field_opts} ->
        field_type = Keyword.get(field_opts, :type)

        if numeric_type?(field_type) do
          :ok
        else
          {:stop,
           {:invalid_config,
            "output_field #{inspect(output_field)} must be a numeric type, got #{inspect(field_type)}"}}
        end

      :error ->
        {:stop,
         {:invalid_config,
          "output_field #{inspect(output_field)} not found in #{inspect(output_message)}"}}
    end
  end

  defp numeric_type?(:float), do: true
  defp numeric_type?(:integer), do: true
  defp numeric_type?(:pos_integer), do: true
  defp numeric_type?(:neg_integer), do: true
  defp numeric_type?(:non_neg_integer), do: true
  defp numeric_type?({:or, types}), do: Enum.any?(types, &numeric_type?/1)
  defp numeric_type?(_), do: false

  defp extract_value(payload, path) do
    get_in(payload, build_accessors(path))
  end

  defp build_accessors(path) do
    Enum.map(path, fn
      key when is_atom(key) -> Access.key!(key)
      index when is_integer(index) -> Access.at!(index)
    end)
  end

  defp build_output_message(output_value, state) do
    BB.Message.new!(state.output_message, state.output_frame_id, [
      {state.output_field, output_value}
    ])
  end

  defp schedule_tick(rate) do
    interval_ms = div(1000, rate)
    Process.send_after(self(), :tick, interval_ms)
  end
end
