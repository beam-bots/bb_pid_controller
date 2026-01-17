<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB PID Controller provides a general-purpose PID controller implementation for Beam Bots robots. It implements the `BB.Controller` behaviour and uses the `pid_control` hex package for the control algorithm.

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

One controller instance = one PID loop. Instantiate multiple controllers for multiple control loops.

## Key Components

| File | Purpose |
|------|---------|
| `lib/bb/pid/controller.ex` | Main PID controller implementation |
| `test/bb/pid/controller_test.exs` | Comprehensive test suite |

## Configuration Options

### PID Gains
- `kp` (required) - Proportional gain
- `ki` (default: 0.0) - Integral gain
- `kd` (default: 0.0) - Derivative gain
- `tau` (default: 1.0) - Derivative low-pass filter (0-1, 1=no filter)
- `output_min` (default: -1.0) - Output clamp minimum
- `output_max` (default: 1.0) - Output clamp maximum

### Setpoint Subscription
- `setpoint_topic` - Topic path to subscribe to (e.g., `[:actuator, :base_link, :joint1, :pid]`)
- `setpoint_message` - Message module to filter for (e.g., `BB.Message.Actuator.Command.Position`)
- `setpoint_path` - Path to value in payload (e.g., `[:position]` or `[:positions, 0]`)

### Measurement Subscription
- `measurement_topic` - Topic path to subscribe to
- `measurement_message` - Message module to filter for
- `measurement_path` - Path to value in payload

### Output Publication
- `output_topic` - Topic path to publish output to
- `output_message` - Message module to construct (e.g., `BB.Message.Actuator.Command.Velocity`)
- `output_field` - Field name for output value (e.g., `:velocity`)
- `output_frame_id` - frame_id for constructed messages

### Control Loop
- `rate` (default: 100) - Control loop frequency in Hz

## Path Extraction

The `*_path` options support atoms (field names) and integers (list indices):

```elixir
[:position]                  # payload.position
[:positions, 0]              # payload.positions |> Enum.at(0)
[:data, :readings, 0, :value] # payload.data.readings[0].value
```

## Example Usage

```elixir
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

## Validation

The controller validates configuration at init time:

1. **Unique sources**: `{setpoint_topic, setpoint_message}` must differ from `{measurement_topic, measurement_message}`
2. **Non-empty paths**: `setpoint_path` and `measurement_path` cannot be empty
3. **Valid output field**: `output_field` must exist in `output_message` schema and be numeric

## Common Commands

```bash
mix check --no-retry    # Run all checks
mix test                # Run tests
mix format              # Format code
```

## Dependencies

- `bb` - Beam Bots core framework
- `pid_control` - PID control algorithm implementation

