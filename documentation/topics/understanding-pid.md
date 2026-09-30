<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Understanding PID

A feedback controller is a simple idea behind an intimidating name. This page explains what the three terms do, how to choose which of them you need, and how to tune them by watching the machine rather than by doing maths.

It doesn't cover how the control law is implemented — for that, including the two places this library deliberately departs from the textbook form, read the `BB.PID.Kernel` moduledoc. If "controller" and "actuator" aren't familiar words yet, start with [Robotics Vocabulary](https://hexdocs.pm/bb/robotics-vocabulary.html).

## Error

You know what you want. You can measure what you've got. The difference between them is the **error**, and the whole question is what command to send in response.

```
error = setpoint - measurement
```

The classic answer has three parts, and you pick the ones you need.

## P, I and D

**P — proportional.** Send a command proportional to the error. Twice as far off, push twice as hard. This does most of the work in most loops.

On its own, P overshoots. It drives hard until it reaches the target, sails past because it was still moving when it got there, comes hard back, and rings like a struck bell.

**D — derivative.** Look at how fast the error is closing and ease off when it's closing quickly. That's damping, and it's the term that turns ringing into settling.

**I — integral.** Keep a running total of the error. If you've been half a degree off for ten seconds, that total grows until the controller does something about it. It exists for the standing offsets that P alone will never quite kill — with P only, the command goes to zero exactly when the error does, so anything that needs a permanent non-zero command to hold position will sit permanently short of the target.

That's it. That's PID.

## Which ones you need

Most controllers use a subset, and which subset is a design decision rather than an oversight.

- **P alone** is fine when a bit of steady-state error doesn't matter and there's enough friction or damping in the system to stop it ringing.
- **PI** is the workhorse for things that settle: temperature, flow, motor speed. No D, because the measurement is noisy and the process is slow enough not to need it.
- **PD** is what you want for something that has to be caught rather than settled — a balancing robot, a fast positioning axis. The I term is often left out deliberately, for reasons in the next section.
- **PID** when you need all three and can afford to tune all three.

In this library, `ki` and `kd` default to `0.0`, so leaving one out means not setting it.

## Putting the integrator somewhere else

There's a case worth knowing about, because it looks like "no integral term" and isn't.

Sometimes the stubborn error isn't in the output — it's in the **setpoint**. A balancing robot is the clearest example: it doesn't balance at zero lean, it balances wherever its centre of mass sits over the wheels, and you can't measure that well enough to write it down. Being wrong about it by a millimetre moves the true balance point by more than a degree.

Hold the wrong lean and the robot doesn't wobble, it accelerates — gently, forever. So the setpoint error isn't cosmetic.

An integrator on the lean error won't fix it, because the loop is already doing exactly what it was told. What does fix it is the observation that at the *true* setpoint, the controller needs no net output to stay there. So if the output is persistently biased one way, the setpoint must be wrong. Integrate the output, and use it to nudge the setpoint.

That's still an I term. It's just integrating the output rather than the error, and correcting the target rather than the command.

Two things make it work:

- It has to be **slow** — seconds, on a machine that falls over in a tenth of one. An integrator running at the speed of the loop it's correcting is just a badly tuned integrator, and it will fight the loop and oscillate.
- It needs a **limit**. If the trim walks out to its bound and stays there, that's a useful signal: something physical has moved, and no amount of tuning will fix it.

## Tuning by watching

Four symptoms, and which way to go:

| What it's doing | Try |
|---|---|
| Sluggish, slow to close the gap | **P** up |
| Rocking, overshooting each way | **D** up |
| Buzzing, chattering, audibly busy while at rest | **D** down |
| Sharp, twitchy, sawing back and forth | **P** down |

A few notes on reading that table.

**Buzzing is a D problem.** The derivative term works off the rate of change of the measurement, and the measurement has noise in it. Turn D up far enough and you're amplifying that noise straight into the output. If the loop sounds busy while the machine is sitting still, that's the sign. The `tau` option low-pass filters the derivative and exists for exactly this; it's a gentler fix than backing D off if you need the damping.

**Rocking and twitching are the same family.** Too much P looks a great deal like not enough D, because what governs the behaviour is the *ratio* between them rather than either number alone. If it's oscillating and you can't tell which, put D up first. If that improves it, that was the problem. If it starts buzzing instead, P was too high all along.

**Not everything is a gain problem.** If the loop is stable but sitting at an offset, or a setpoint trim has crept out to its limit, that's telling you something about the machine and not about your numbers. Check the physical thing before touching P or D.

## How far to move

Change one gain at a time, and don't creep. A five percent change disappears into the noise and teaches you nothing.

Treat it as a binary search, because that's what it is. Each gain has a plausible range, and somewhere in it is the good part. So halve it or double it. Go far enough that it clearly gets *worse* — now you have a bracket with a known-bad end and a known-better end, and you can bisect.

Four confident jumps will beat twenty timid nudges, and every one of them tells you something.

This is also the argument for declaring your gains as BB parameters:

```elixir
controller :shoulder_pid, {BB.PID.Controller,
  kp: param([:shoulder, :kp]),
  kd: param([:shoulder, :kd]),
  # ...
}
```

Parameters can be changed on a running robot, so you can try a value, watch what happens, and try another without a rebuild. If each attempt costs you a firmware build you'll try about four of them. This way you'll try forty, and forty is roughly what it takes.

## Rate

`rate` sets how often the loop runs, in hertz. Faster isn't automatically better — the loop can only be as good as the measurement feeding it, and running at 500 Hz off a sensor publishing at 100 Hz just means four out of every five ticks act on a stale reading.

`ki` and `kd` are gains per second, applied against the measured time delta rather than an assumed one, so changing `rate` doesn't silently rescale your tuning.

## Further reading

- `BB.PID.Kernel` — the control law itself: derivative-on-measurement, back-calculation anti-windup, and the batched form
- [Getting Started](../tutorials/01-getting-started.md) — wiring a controller into a robot
- [Robotics Vocabulary](https://hexdocs.pm/bb/robotics-vocabulary.html) — controllers, actuators and the rest of the BB glossary
