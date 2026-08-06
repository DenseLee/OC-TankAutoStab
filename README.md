# CC:Tweaked Tank Tussle stabilizer

`tank_stabilizer.lua` is a world-space turret stabilizer for a ComputerCraft computer mounted on a separate VS: Clockwork turret ship.

## Wiring assumed by the default configuration

```text
                 Tweaked Lectern Controller
                              |
                    [computer] -- front modem
                     /       \
       left resistor         right resistor
        left gearshift        right gearshift
```

The program captures the turret ship's current CC:VS quaternion on startup and holds that world-space direction. It uses four control paths:

- computer left → horizontal Redstone Resistor (analog speed)
- computer right → vertical Redstone Resistor (analog speed)
- front-network Redstone Relay left → horizontal gearshift (binary direction)
- front-network Redstone Relay right → vertical gearshift (binary direction)

Install and run:

```text
pastebin get <your-pastebin-id> tank_stabilizer
tank_stabilizer
```

Or copy the file directly to the computer as `tank_stabilizer`.

## Stabilizer behavior

On startup, the program stores the turret's current world orientation. When the tank hull rotates, the turret's current quaternion changes and the program drives it back to the stored orientation.

- left output: horizontal correction
- right output: vertical correction
- Redstone Resistor speed mapping: `0` = full speed; `15` = stopped

The program drives the Redstone Relay's left/right sides to reverse the corresponding Create gearshift when the yaw/pitch error changes sign. It starts with both resistors at `15` and both gearshifts unpowered whenever the turret is at the target angle or the program stops.

The control loop runs once per game tick with `sleep(0.05)`.

The current tuning uses `fullSpeedAtDegrees = 45`, a `0.2` degree deadzone, proportional gain `1`, and derivative gain `0.0005`. There is deliberately no reversal delay or integral term.

Each resistor uses a one-tick (`0.05` second) output cooldown by default. Direction signals remain immediate.

## Player-controlled target

Place a Tweaked Controller on the lectern behind the computer. CC:Tweaked exposes it as a `tweaked_controller` peripheral; the program finds it automatically, including through the wired-modem network. While a player is using the lectern, the right stick adjusts the saved target:

- right-stick X (axis 3): horizontal target aim
- right-stick Y (axis 4): vertical target aim

When the player releases the lectern or centers the stick, the turret holds the new world-space target. `aimDegreesPerSecond` controls manual turn speed; toggle `invertHorizontalAim` or `invertVerticalAim` if either input feels reversed.

## Fault-isolation stress test

`stability_stress_test.lua` helps identify why a resistor breaks. It tests one axis at a time and always stops both resistors/unpowers both gearshifts on exit.

Start with `mode = "resistor"`: it changes only the resistor's analogue signal while its gearshift stays in one direction. If that breaks the resistor, redstone-level churn or fractional RPM is sufficient to cause the fault.

Then use `mode = "gearshift"`: it holds the resistor at a fixed speed while reversing only the gearshift. If that breaks the resistor, reversing under transmitted rotation is the cause.

Finally use `mode = "both"` only if neither isolated test breaks it. It recreates simultaneous speed and direction changes. Begin with the default five-second duration and change `CHANGE_EVERY_TICKS` to a larger number for a less aggressive test.

## Tuning

Edit the `CONFIG` table at the top of the program:

- swap `horizontalOutputSide` and `verticalOutputSide` only if the physical axis wiring is reversed;
- if an axis initially drives farther from its stored direction, toggle that axis's `...ReverseOnPositiveError` setting;
- increase `fullSpeedAtDegrees` for a gentler correction, or decrease it for a faster correction;
- `minimumRedstoneResistorLevel` caps maximum transmission. The current value is `0`, as tuned for your setup.
- increase `deadzone` if the controller drifts at center;
- set `showDebug = false` once the setup works.

## Controller cable input test

`controller_input_test.lua` is a safe diagnostic for a remote controller setup. It drives nothing: it only displays the four analogue redstone inputs every `0.05` seconds.

- computer top: horizontal `+X`
- computer back: horizontal `-X`
- Redstone Relay top: elevation `+Y`
- Redstone Relay bottom: elevation `-Y`

It is intended for separate positive/negative Drive-By-Wire-to-redstone outputs. Run this test before connecting those inputs to the stabilizer's manual-aim controls.
