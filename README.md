<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots PID Controller

[![CI](https://github.com/beam-bots/bb_pid_controller/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_pid_controller/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_pid_controller.svg)](https://hex.pm/packages/bb_pid_controller)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_pid_controller)](https://api.reuse.software/info/github.com/beam-bots/bb_pid_controller)

A general-purpose PID controller for the [Beam Bots](https://github.com/beam-bots/bb) robotics framework.

This library provides a `BB.Controller` implementation that subscribes to configurable topics for setpoint and measurement values, runs a periodic PID loop, and publishes output to a configurable topic. One controller instance = one PID loop.

## Features

- **Configurable subscriptions** - subscribe to any topic/message combination for setpoint and measurement
- **Flexible path extraction** - extract values from nested message fields or list indices
- **Configurable output** - publish to any topic with any numeric message field
- **Runtime parameter updates** - PID gains can be changed at runtime via BB parameters
- **Validation** - validates configuration at init time with clear error messages

## Installation

Add `bb_pid_controller` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_pid_controller, "~> 0.2.0"}
  ]
end
```

## Requirements

- BB framework (`~> 0.13`)

## Usage

Define a PID controller in your robot DSL:

```elixir
defmodule MyRobot do
  use BB

  controller :shoulder_pid, {BB.PID.Controller,
    kp: 2.0, ki: 0.1, kd: 0.05,
    output_min: -1.0, output_max: 1.0,

    # Subscribe to position commands
    setpoint_topic: [:actuator, :base_link, :shoulder, :pid],
    setpoint_message: BB.Message.Actuator.Command.Position,
    setpoint_path: [:position],

    # Subscribe to encoder feedback
    measurement_topic: [:sensor, :base_link, :shoulder, :encoder],
    measurement_message: BB.Message.Sensor.JointState,
    measurement_path: [:positions, 0],

    # Publish velocity commands to servo
    output_topic: [:actuator, :base_link, :shoulder, :servo],
    output_message: BB.Message.Actuator.Command.Velocity,
    output_field: :velocity,
    output_frame_id: :shoulder,

    rate: 100
  }

  topology do
    link :base_link do
      joint :shoulder, type: :revolute do
        link :upper_arm do
        end
      end
    end
  end
end
```

## Sending Setpoints

Send position setpoints to the PID controller via pubsub:

```elixir
{:ok, msg} = BB.Message.new(
  BB.Message.Actuator.Command.Position,
  :shoulder,
  position: 1.57
)

BB.PubSub.publish(MyRobot, [:actuator, :base_link, :shoulder, :pid], msg)
```

The controller will compute the PID output and publish velocity commands to the servo.

## Configuration Options

### PID Gains

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `kp` | float | required | Proportional gain |
| `ki` | float | 0.0 | Integral gain |
| `kd` | float | 0.0 | Derivative gain |
| `tau` | float | 1.0 | Derivative low-pass filter (0-1, 1=no filter) |
| `output_min` | float | -1.0 | Output clamp minimum |
| `output_max` | float | 1.0 | Output clamp maximum |

### Setpoint Subscription

| Option | Type | Description |
|--------|------|-------------|
| `setpoint_topic` | `[atom]` | Topic path to subscribe to |
| `setpoint_message` | module | Message module to filter for |
| `setpoint_path` | `[atom \| integer]` | Path to value in payload |

### Measurement Subscription

| Option | Type | Description |
|--------|------|-------------|
| `measurement_topic` | `[atom]` | Topic path to subscribe to |
| `measurement_message` | module | Message module to filter for |
| `measurement_path` | `[atom \| integer]` | Path to value in payload |

### Output Publication

| Option | Type | Description |
|--------|------|-------------|
| `output_topic` | `[atom]` | Topic path to publish to |
| `output_message` | module | Message module to construct |
| `output_field` | atom | Field name for output value |
| `output_frame_id` | atom | frame_id for constructed messages |

### Control Loop

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `rate` | pos_integer | 100 | Control loop frequency (Hz) |

## Path Extraction

The `*_path` options support atoms (field names) and integers (list indices):

```elixir
# Simple field access
setpoint_path: [:position]           # payload.position

# List index access
measurement_path: [:positions, 0]    # payload.positions |> Enum.at(0)

# Nested access
path: [:data, :readings, 0, :value]  # payload.data.readings[0].value
```

## How It Works

### Architecture

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

### Message Flow

1. Controller subscribes to `setpoint_topic` and `measurement_topic` at init
2. When a message arrives on `setpoint_topic` with matching `setpoint_message` type, the setpoint value is extracted and stored
3. When a message arrives on `measurement_topic` with matching `measurement_message` type, the measurement value is extracted and stored
4. Every `1000/rate` ms, if both setpoint and measurement exist:
   - PID step is computed: `output = Kp*error + Ki*integral + Kd*derivative`
   - Output is clamped to `[output_min, output_max]`
   - Output message is constructed and published to `output_topic`

### Validation

The controller validates configuration at init time:

1. **Unique sources** - `{setpoint_topic, setpoint_message}` must differ from `{measurement_topic, measurement_message}`
2. **Non-empty paths** - `setpoint_path` and `measurement_path` cannot be empty
3. **Valid output field** - `output_field` must exist in `output_message` schema and be numeric

## Common Use Cases

### Position Control with Velocity Output

Use encoder feedback to control position, outputting velocity commands:

```elixir
controller :joint_pid, {BB.PID.Controller,
  kp: 5.0, ki: 0.5, kd: 0.1,
  setpoint_topic: [:actuator, :joint, :pid],
  setpoint_message: BB.Message.Actuator.Command.Position,
  setpoint_path: [:position],
  measurement_topic: [:sensor, :joint, :encoder],
  measurement_message: BB.Message.Sensor.JointState,
  measurement_path: [:positions, 0],
  output_topic: [:actuator, :joint, :motor],
  output_message: BB.Message.Actuator.Command.Velocity,
  output_field: :velocity,
  output_frame_id: :joint
}
```

### Velocity Control with Effort Output

Use velocity feedback to control velocity, outputting effort commands:

```elixir
controller :velocity_pid, {BB.PID.Controller,
  kp: 1.0, ki: 0.1,
  setpoint_topic: [:actuator, :wheel, :velocity_cmd],
  setpoint_message: BB.Message.Actuator.Command.Velocity,
  setpoint_path: [:velocity],
  measurement_topic: [:sensor, :wheel, :encoder],
  measurement_message: BB.Message.Sensor.JointState,
  measurement_path: [:velocities, 0],
  output_topic: [:actuator, :wheel, :motor],
  output_message: BB.Message.Actuator.Command.Effort,
  output_field: :effort,
  output_frame_id: :wheel
}
```

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_pid_controller).
