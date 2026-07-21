<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.PID.Controller Usage Rules

`bb_pid_controller` provides `BB.PID.Controller`, a general-purpose PID
implementation of the `BB.Controller` behaviour for
[Beam Bots](https://hexdocs.pm/bb). For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
the controller.

## Core principles

1. **It is a declared, supervised controller — not a solver you pass.** Wire it
   into the `controllers` block as `{BB.PID.Controller, opts}`. BB wraps it in a
   supervised process; you never write a `child_spec`.
2. **It is entirely message-driven.** The controller subscribes to a setpoint
   topic and a measurement topic, and publishes its output to a third topic. It
   does *not* touch actuators, joints, or the topology directly — everything is
   PubSub. One controller instance is exactly one PID loop; declare several for
   several loops.
3. **It runs its own tick loop at `rate` Hz** (`Process.send_after/3` +
   `handle_info(:tick, …)`). Each tick it publishes only once *both* a setpoint
   and a measurement have arrived; until then it stays quiet.

## Wiring it in

```elixir
controllers do
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
    rate: 100}
end
```

`mix igniter.install bb_pid_controller` scaffolds this entry with `kp`/`ki`/`kd`
wired to tunable parameters and the topic paths left as `[:TODO]` to fill in.

## Options

| Option | Default | Meaning |
|---|---|---|
| `kp` | required | Proportional gain |
| `ki` | `0.0` | Integral gain |
| `kd` | `0.0` | Derivative gain |
| `tau` | `1.0` | Derivative low-pass filter (`0`–`1`, `1` = no filter) |
| `output_min` / `output_max` | `-1.0` / `1.0` | Output clamp bounds |
| `setpoint_topic` / `setpoint_message` / `setpoint_path` | required | Where the target comes from, the message type to match, and the path to the value in its payload |
| `measurement_topic` / `measurement_message` / `measurement_path` | required | Same three for the feedback signal |
| `output_topic` / `output_message` / `output_field` / `output_frame_id` | required | Topic to publish to, message type to build, numeric field for the output, and its `frame_id` |
| `rate` | `100` | Loop frequency in Hz (tick interval is `div(1000, rate)` ms) |

A `*_path` is a list of atoms (field names) and integers (list indices):
`[:position]`, `[:positions, 0]`, `[:data, :readings, 0, :value]`.

The gains are plain floats, but you can point them at robot parameters —
`kp: param([:config, :shoulder_pid, :kp])` — to make them tunable at runtime;
`handle_options/2` rebuilds the PID state when a parameter changes.

## Anti-patterns

- **Don't expect it to run in simulation by default.** DSL-declared controllers
  default to `simulation: :omit`, so a robot started in simulation mode does
  *not* start this controller. Add `simulation: :mock` or `simulation: :start`
  to the entry if you need it under simulation.
- **Don't give the setpoint and measurement the same source.** Init fails if
  `{setpoint_topic, setpoint_message}` equals `{measurement_topic,
  measurement_message}`; paths must be non-empty; and `output_field` must name a
  numeric field on `output_message` or init rejects the config.
- **Don't rely on it stopping when the robot disarms.** This controller does not
  implement `disarm/1` and does not register with `BB.Safety` — it keeps ticking
  and publishing regardless of safety state. If its output must cease on disarm,
  that has to come from the downstream actuator, not from here.

## Further reading

- [bb_pid_controller docs](https://hexdocs.pm/bb_pid_controller)
- `bb`'s controller and safety rules (`bb:safety-and-commands`) and
  [Reactive Controllers](https://hexdocs.pm/bb/reactive-controllers.html)
