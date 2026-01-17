<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Getting Started with PID Control

In this tutorial, you'll learn how to add closed-loop PID control to your Beam Bots robot. We'll create a position controller that reads encoder feedback and outputs velocity commands to a motor.

## Prerequisites

- Familiarity with the BB DSL (see the [BB tutorials](https://hexdocs.pm/bb))
- A robot with sensors providing feedback (e.g., encoders)
- An actuator that accepts commands (e.g., a motor driver)

## What is PID Control?

PID (Proportional-Integral-Derivative) control is a feedback control algorithm that continuously calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms.

```
error = setpoint - measurement

output = Kp * error           # Proportional: react to current error
       + Ki * ∫error dt       # Integral: eliminate steady-state error
       + Kd * d(error)/dt     # Derivative: dampen oscillations
```

## Installation

Add `bb_pid_controller` to your dependencies:

```elixir
def deps do
  [
    {:bb, "~> 0.13"},
    {:bb_pid_controller, "~> 0.2"}
  ]
end
```

Run `mix deps.get`.

## Adding a PID Controller

Let's add a PID controller to control the position of a joint. The controller will:

1. Subscribe to position setpoints (where we want the joint to be)
2. Subscribe to encoder feedback (where the joint actually is)
3. Compute the PID output
4. Publish velocity commands to the motor

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :shoulder_pid, {BB.PID.Controller,
      # PID gains
      kp: 2.0,
      ki: 0.1,
      kd: 0.05,

      # Output limits (rad/s for velocity)
      output_min: -2.0,
      output_max: 2.0,

      # Where to get the setpoint
      setpoint_topic: [:actuator, :base_link, :shoulder, :pid],
      setpoint_message: BB.Message.Actuator.Command.Position,
      setpoint_path: [:position],

      # Where to get the measurement
      measurement_topic: [:sensor, :base_link, :shoulder, :encoder],
      measurement_message: BB.Message.Sensor.JointState,
      measurement_path: [:positions, 0],

      # Where to send the output
      output_topic: [:actuator, :base_link, :shoulder, :motor],
      output_message: BB.Message.Actuator.Command.Velocity,
      output_field: :velocity,
      output_frame_id: :shoulder,

      # Control loop rate
      rate: 100
    }
  end

  topology do
    link :base_link do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree)

        # The motor actuator receives velocity commands from the PID
        actuator :motor, MyMotorDriver

        # The encoder provides position feedback
        sensor :encoder, MyEncoder

        link :upper_arm do
        end
      end
    end
  end
end
```

## Understanding the Configuration

### Setpoint Configuration

The setpoint is the target value we want to achieve. Here we're subscribing to position commands:

```elixir
setpoint_topic: [:actuator, :base_link, :shoulder, :pid],
setpoint_message: BB.Message.Actuator.Command.Position,
setpoint_path: [:position],
```

- `setpoint_topic` - The pubsub topic to listen on
- `setpoint_message` - Only process messages of this type
- `setpoint_path` - Extract the value from `payload.position`

### Measurement Configuration

The measurement is the actual current value from our sensor:

```elixir
measurement_topic: [:sensor, :base_link, :shoulder, :encoder],
measurement_message: BB.Message.Sensor.JointState,
measurement_path: [:positions, 0],
```

The path `[:positions, 0]` extracts the first element from the `positions` list in the JointState message.

### Output Configuration

The output is where we send our computed control signal:

```elixir
output_topic: [:actuator, :base_link, :shoulder, :motor],
output_message: BB.Message.Actuator.Command.Velocity,
output_field: :velocity,
output_frame_id: :shoulder,
```

The controller constructs a `Velocity` message with the PID output in the `velocity` field.

## Sending Setpoints

To move the joint to a target position, publish a position command to the PID's setpoint topic:

```elixir
# Start the robot
{:ok, _} = MyRobot.start_link()

# Create a position command (1.0 radian ≈ 57 degrees)
{:ok, msg} = BB.Message.new(
  BB.Message.Actuator.Command.Position,
  :shoulder,
  position: 1.0
)

# Publish to the PID controller's setpoint topic
BB.PubSub.publish(MyRobot, [:actuator, :base_link, :shoulder, :pid], msg)
```

The PID controller will now continuously:
1. Read the current position from the encoder
2. Calculate the error (setpoint - measurement)
3. Compute the PID output
4. Publish velocity commands to the motor

## Monitoring the Control Loop

Subscribe to the output topic to see what the PID controller is publishing:

```elixir
BB.PubSub.subscribe(MyRobot, [:actuator, :base_link, :shoulder, :motor])

# Check for messages
flush()
# {:bb, [:actuator, :base_link, :shoulder, :motor],
#  %BB.Message{payload: %BB.Message.Actuator.Command.Velocity{velocity: 0.5}}}
```

## Tuning PID Gains

Finding the right PID gains requires experimentation. Here's a simple tuning procedure:

### 1. Start with P only

Set `ki: 0` and `kd: 0`. Increase `kp` until the system responds quickly but oscillates:

```elixir
kp: 1.0, ki: 0.0, kd: 0.0
```

### 2. Add D to reduce oscillation

Increase `kd` to dampen oscillations:

```elixir
kp: 1.0, ki: 0.0, kd: 0.1
```

### 3. Add I to eliminate steady-state error

If the system settles slightly off target, add integral gain:

```elixir
kp: 1.0, ki: 0.05, kd: 0.1
```

### Common Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Slow response | `kp` too low | Increase `kp` |
| Oscillation | `kp` too high or `kd` too low | Decrease `kp` or increase `kd` |
| Overshoot | `kd` too low | Increase `kd` |
| Never reaches target | `ki` too low | Increase `ki` |
| Windup/instability | `ki` too high | Decrease `ki` or reduce output limits |

## Path Extraction Examples

The `*_path` options are flexible. Here are some examples:

### Simple Field Access

```elixir
# payload.position
setpoint_path: [:position]
```

### List Index

```elixir
# payload.positions[0]
measurement_path: [:positions, 0]

# payload.velocities[2]
measurement_path: [:velocities, 2]
```

### Nested Access

```elixir
# payload.data.sensors[0].value
measurement_path: [:data, :sensors, 0, :value]
```

## Multiple PID Controllers

You can have multiple PID controllers for different joints:

```elixir
controllers do
  controller :shoulder_pid, {BB.PID.Controller,
    kp: 2.0, ki: 0.1, kd: 0.05,
    setpoint_topic: [:actuator, :base, :shoulder, :pid],
    # ... rest of config
  }

  controller :elbow_pid, {BB.PID.Controller,
    kp: 1.5, ki: 0.2, kd: 0.08,
    setpoint_topic: [:actuator, :base, :shoulder, :elbow, :pid],
    # ... rest of config
  }

  controller :wrist_pid, {BB.PID.Controller,
    kp: 1.0, ki: 0.1, kd: 0.05,
    setpoint_topic: [:actuator, :base, :shoulder, :elbow, :wrist, :pid],
    # ... rest of config
  }
end
```

Each controller runs its own independent control loop.

## Cascaded Control

For more sophisticated control, you can cascade controllers. For example, an outer position loop feeding an inner velocity loop:

```elixir
controllers do
  # Outer loop: position -> velocity setpoint
  controller :position_pid, {BB.PID.Controller,
    kp: 2.0,
    setpoint_topic: [:cmd, :position],
    setpoint_message: BB.Message.Actuator.Command.Position,
    setpoint_path: [:position],
    measurement_topic: [:sensor, :encoder],
    measurement_message: BB.Message.Sensor.JointState,
    measurement_path: [:positions, 0],
    output_topic: [:cmd, :velocity],  # Feeds the inner loop
    output_message: BB.Message.Actuator.Command.Velocity,
    output_field: :velocity,
    output_frame_id: :joint,
    rate: 50
  }

  # Inner loop: velocity -> effort
  controller :velocity_pid, {BB.PID.Controller,
    kp: 0.5, ki: 0.1,
    setpoint_topic: [:cmd, :velocity],  # From outer loop
    setpoint_message: BB.Message.Actuator.Command.Velocity,
    setpoint_path: [:velocity],
    measurement_topic: [:sensor, :encoder],
    measurement_message: BB.Message.Sensor.JointState,
    measurement_path: [:velocities, 0],
    output_topic: [:actuator, :motor],
    output_message: BB.Message.Actuator.Command.Effort,
    output_field: :effort,
    output_frame_id: :joint,
    rate: 200  # Inner loop runs faster
  }
end
```

## What's Next?

You now know how to:
- Add a PID controller to your robot
- Configure setpoint, measurement, and output topics
- Tune PID gains
- Use path extraction for nested values
- Set up multiple and cascaded controllers

For more details, see the [API documentation](BB.PID.Controller.html).
