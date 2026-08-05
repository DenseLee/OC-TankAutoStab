[README.md](https://github.com/user-attachments/files/30756372/README.md)
# OC-TankAutoStab# CC:Tweaked Tank Tussle stabilizer

`tank_stabilizer.lua` is a standalone ComputerCraft program for a computer on a Create/Clockwork tank.

## Wiring assumed by the default configuration

```text
                 Tweaked Lectern Controller
                              |
                    [computer] -- front modem
                     /       \
       left resistor         right resistor
        left gearshift        right gearshift
```

The program writes analog redstone levels to the computer's `left` and `right` sides. The front modem is optional if the lectern is directly connected; otherwise it lets CC:Tweaked discover the lectern over the peripheral network. The lectern must appear as the `tweaked_controller` peripheral.

Install and run:

```text
pastebin get <your-pastebin-id> tank_stabilizer
tank_stabilizer
```

Or copy the file directly to the computer as `tank_stabilizer`.

## Controller behavior

The default `differential` mode uses the left stick:

- left stick Y: forward/reverse throttle
- left stick X: steering
- output: left = throttle + steering; right = throttle - steering

The output is a magnitude from 0 to 15. The mechanical Create/Clockwork arrangement must provide the desired direction and gearing; the script does not reverse a gearshift by itself.

For a one-axis stabilizer pair, change:

```lua
mode = "single_axis"
```

That mode reads the right-stick X axis and drives only one side at a time: positive correction drives the left output, negative correction drives the right output.

## Tuning

Edit the `CONFIG` table at the top of the program:

- swap `leftOutputSide` and `rightOutputSide` if the tank steers backward;
- change `invertThrottle`, `invertSteering`, or `invertStabilizer` if an axis is reversed;
- increase `deadzone` if the controller drifts at center;
- set `showDebug = false` once the setup works.

The Tweaked Lectern Controller peripheral API provides `hasUser()`, `getAxis(1..6)`, and `getButton(1..15)`. This program uses `getAxis` and automatically shuts both outputs off when no player is using the controller.

## Important limitation

True turret auto-stabilization requires a heading/rotation sensor or a second feedback signal. A controller alone can provide a correction command, but it cannot know how much the tank hull has rotated. The included `single_axis` mode is therefore an open-loop two-gearshift stabilizer/control pair, not a closed-loop gyro stabilizer.
