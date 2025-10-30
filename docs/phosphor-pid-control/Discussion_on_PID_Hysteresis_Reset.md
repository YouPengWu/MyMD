# Discussion on PID Hysteresis reset

This document was inspired by a section of code I found in pidcontroller.cpp
within the phosphor-pid-control package of OpenBMC,

## Code Context in pidcontroller.cpp

The section I want to discuss mainly focuses on the following condition: else if
(input < (setpt - info->negativeHysteresis)),The following is the code content :

```cpp
if (info->checkHysterWithSetpt)
{
    // Over the hysteresis bounds, keep counting pid
    if (input > (setpt + info->positiveHysteresis))
    {
        // Calculate new output
        output = ec::pid(info, input, setpt, &name);

        // this variable isn't actually used in this context, but we're
        // setting it here in case somebody uses it later it's the correct
        // value
        lastInput = input;
    }
    // Under the hysteresis bounds, initialize pid
    else if (input < (setpt - info->negativeHysteresis))
    {
        lastInput = setpt;
        info->integral = 0;
        output = 0;
    }
    // inside the hysteresis bounds, keep last output
    else
    {
        lastInput = input;
        output = info->lastOutput;
    }

    info->lastOutput = output;
}
else
```

I believe that when the input leaves the lower edge of the hysteresis region, it
is not appropriate to immediately reset the output and the accumulated integral
term to zero. Instead, allowing them to gradually decay to zero over time is
more reasonable and better reflects the behavior of a true PID controller.

## Simulation Example

For example assuming Setpoint = 83, $K_p$ = -5, $K_i$ = -2, $K_d$ = 0,
$\Delta t$ = 1, NegativeHysteresis = 1, PositiveHysteresis = 1.

| Time | Input                   | $e(t)$ | Pterm(time-based decay ) | Pterm(instant reset, in the community) | Iterm(time-based decay ) | Iterm(instant reset, in the community) |
| ---- | ----------------------- | ------ | ------------------------ | -------------------------------------- | ------------------------ | -------------------------------------- |
| `0`  | `85` In upper bound     | $-2$   | 10                       | 10                                     | 4                        | 4                                      |
| `1`  | `85` In upper bound     | $-2$   | 10                       | 10                                     | 8                        | 8                                      |
| `2`  | `85` In upper bound     | $-2$   | 10                       | 10                                     | 12                       | 12                                     |
| `3`  | `85` In upper bound     | $-2$   | 10                       | 10                                     | 16                       | 16                                     |
| `4`  | `85` In upper bound     | $-2$   | 10                       | 10                                     | 20                       | 20                                     |
| `5`  | `84` In hysteresis band | $-1$   | 10(same as previous)     | 10 (same as previous)                  | 20(same as previous)     | 20 (same as previous)                  |
| `6`  | `83` In hysteresis band | $0$    | 10(same as previous)     | 10 (same as previous)                  | 20(same as previous)     | 20 (same as previous)                  |
| `7`  | `82` In hysteresis band | $1$    | 10(same as previous)     | 10 (same as previous)                  | 20(same as previous)     | 20 (same as previous)                  |
| `8`  | `81` In lower bound     | $2$    | -10                      | 0(reset)                               | 16                       | 0(reset)                               |
| `9`  | `82` In hysteresis band | $1$    | -10(same as previous)    | 0(same as previous)                    | 16 (same as previous)    | 0(same as previous)                    |
| `10` | `81` In lower bound     | $2$    | -10                      | 0(reset)                               | 12                       | 0(reset)                               |
| `11` | `81` In lower bound     | $2$    | -10                      | 0(reset)                               | 8                        | 0(reset)                               |
| `12` | `81` In lower bound     | $2$    | -10                      | 0(reset)                               | 4                        | 0(reset)                               |

| Time | Output(time-based decay ) | Output(instant reset, in the community) |
| ---- | ------------------------- | --------------------------------------- |
| `0`  | 14                        | 14                                      |
| `1`  | 18                        | 18                                      |
| `2`  | 22                        | 22                                      |
| `3`  | 26                        | 26                                      |
| `4`  | 30                        | 30                                      |
| `5`  | 30(same as previous)      | 30 (same as previous)                   |
| `6`  | 30(same as previous)      | 30 (same as previous)                   |
| `7`  | 30(same as previous)      | 30 (same as previous)                   |
| `8`  | 6                         | 0(reset)                                |
| `9`  | 6(same as previous)       | 0(same as previous)                     |
| `10` | 2                         | 0(reset)                                |
| `11` | 0                         | 0(reset)                                |
| `12` | 0                         | 0(reset)                                |

## Conclusion and Thoughts

I suspect that the original design, which reset both the controller output and
the integral term to zero, was intended to prevent integral wind-up. However,
since the controller already defines two parameters `ILimitMax` = 100 and
`ILimitMin` = 0 that effectively constrain the upper and lower bounds of the
integral term, a full reset is no longer necessary.

We can also observe that in the current community implementation, where the
output is instantly reset, a large transient appears between time 7 and 8 as a
result of the reset behavior. In some PID configurations, this jump could become
even more pronounced. While reducing the PID coefficients might suppress such
oscillations, it would also slow down convergence. Because a continuously
running PID loop naturally drives the output toward zero over time, it is
therefore more reasonable to continue executing the PID computation when the
input falls below the lower edge of the hysteresis region, rather than resetting
the output and integral term immediately. This gradual, time-based decay better
reflects the behavior of a true PID controller and helps to avoid abrupt
transients when the system leaves the hysteresis band.
