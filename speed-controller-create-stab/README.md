# Create speed-controller stabilizer

This folder is for the next stabilizer design: signed RPM control through a Create Rotation Speed Controller, instead of a Redstone Resistor plus a gearshift.

Start by putting a wired modem on the ComputerCraft computer and on the Create Rotation Speed Controller, then run:

```text
peripheral_check
```

It must list a peripheral whose type includes `Create_RotationSpeedController` and whose methods include `setTargetSpeed`.

Then run the motion test:

```text
speed_controller_sweep
```

It commands one selected speed controller through a smooth linear sweep:

```text
0 -> +256 RPM -> 0 -> -256 RPM -> 0
```

Positive versus negative physical rotation depends on the shaft arrangement. Set `maxRPM` below `256` in `speed_controller_sweep.lua` if the first test should be gentler. `Ctrl+T` and any runtime error command `0 RPM` before the program exits.

If more than one speed controller is connected, set `CONFIG.peripheralName` in the sweep script to the exact peripheral name printed by the check script.

## Computer-assisted driving

`computer_assisted_driving.lua` is for the hull computer's back-mounted transmission Speed Controller. It reads the Tweaked Controller's left-stick Y outputs:

- computer front: forward (`+Y`);
- computer top: reverse (`-Y`), through the independently mounted gearbox;
- computer back: transmission Create Rotation Speed Controller.

It subtracts reverse from forward and commands a signed target speed from `-256` to `+256 RPM`. The `inputCurveExponent` setting gives low stick inputs more precise, slower movement while keeping full stick at full configured speed. Run `Ctrl+T` to stop safely.
