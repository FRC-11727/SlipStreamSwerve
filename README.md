# SlipStream 11727 — MK4n Swerve (NEO Vortex + CTRE sensors, single CAN bus)

Drivetrain-only WPILib **2026** Java (command-based) code for:

| Part | Qty | Notes |
|---|---|---|
| NEO Vortex on SPARK Flex | 8 | 4 drive, 4 steer |
| CTRE CANcoder | 4 | absolute steering angle |
| CTRE Pigeon 2.0 | 1 | gyro |
| SDS MK4n | 4 | steer 18.75:1, drive L1+/L2+/L3+ |
| REV PDH, roboRIO 2.0, radio | 1 each | no CANivore — everything on the roboRIO CAN bus |

Every value you need to change is marked `// >>> EDIT` (almost all in `Constants.java`).

## How it works

The code is split into two layers:

| File | Job |
|---|---|
| `SwerveSubsystem`, `SwerveModule`, `TeleopDrive` | Driving logic. **Shared** by the robot and the simulator. |
| `ModuleIOSparkFlex`, `GyroIOPigeon2` | Talk to the **real** SPARK Flex, CANcoder and Pigeon. |
| `ModuleIOSim`, `GyroIOSim` | **Simulator** physics models. Never used on the robot. |

The robot picks the right layer automatically at startup, so you never switch anything by hand.

- **Drive** = SPARK Flex onboard **velocity PID + feedforward** (kS/kV/kA in volts), in m/s.
- **Steer** = SPARK Flex onboard **position PID** on the Vortex's built-in encoder, in radians, with position wrapping (never spins the long way).
- **CANcoder** seeds the steer encoder at boot and is re-checked every loop. If the wheel is still and the two disagree by >3° for 0.2 s (e.g. a SPARK rebooted during a brownout), it re-seeds automatically. A module whose CANcoder never reports refuses to drive instead of driving at a wrong angle.
- **Pigeon** yaw is latency-compensated with yaw rate and fed to a `SwerveDrivePoseEstimator` (ready for vision later).
- Teleop is field-relative, with deadband, squared inputs, slew limiting, alliance-aware driving, slow mode, X-lock, and an optional heading-hold PID.

## Why this works without a CANivore

All PID loops run **on the SPARK Flex at 1 kHz**, so control quality doesn't depend on how fast data crosses the CAN bus. That lets us cut telemetry hard:

| Source | What we keep | Approx. frames/s |
|---|---|---|
| 8× SPARK Flex encoder frame | position + velocity @ 50 Hz | 400 |
| 8× SPARK Flex telemetry frame | output/voltage/current/temp @ 10 Hz (default is 100 Hz) | 80 |
| 8× SPARK Flex faults/warnings | @ 4 Hz | ~64 |
| 8× setpoints from the roboRIO | 50 Hz | 400 |
| Pigeon 2.0 | yaw + yaw rate @ 100 Hz, **everything else off** | 200 |
| 4× CANcoder | absolute position @ 10 Hz, **everything else off** | 40 |
| PDH + roboRIO heartbeat | fixed | ~100 |
| **Total** | | **~1,300 → roughly 15–20% bus** |

These are estimates (assume ~130 bits per frame on a 1 Mbit/s bus). The real number is on the dashboard as **`CAN Utilization %`** — check it while driving. Under ~60% is healthy.

Things in the code that keep it this low:
- `optimizeBusUtilization()` on the Pigeon and every CANcoder turns off every CTRE signal we didn't explicitly ask for.
- SPARK status frames are set in `applySignalRates()`. **Never** call `getAbsoluteEncoder()`, `getAnalog()` or `getExternalEncoder()` on these SPARKs — REVLib turns frames on when you ask for those.
- All six CTRE signals are read with one non-blocking `BaseStatusSignal.refreshAll(...)` per loop.
- Live gain tuning only sends CAN parameter frames when you actually change a number.

**Wiring tips for one shared bus:** one continuous daisy-chain (no star/stubs longer than a few inches), roboRIO at one end, PDH at the other end with its termination enabled (or a 120 Ω resistor), twisted yellow/green, and crimped/soldered connections — WAGO-style lever nuts work, loose twist-ties don't.

## Project setup

This folder is a complete WPILib project. It already contains the build files, the Gradle wrapper and the vendor libraries, so you don't need to create a project or install anything except WPILib itself.

| What | Version |
|---|---|
| WPILib / GradleRIO | 2026.2.1 |
| REVLib | 2026.0.5 (`vendordeps/REVLib.json`) |
| CTRE Phoenix 6 | 26.3.0 (`vendordeps/Phoenix6-26.3.0.json`) |
| WPILib command library | 2026 (`vendordeps/WPILibNewCommands.json`) |
| Team number | 11727 (`.wpilib/wpilib_preferences.json`) |

1. Install **WPILib 2026.2.1** or newer 2026 release (the last year that supports the roboRIO).
2. Unzip this folder somewhere simple, like `Documents/SlipStreamSwerve`.
3. Open **2026 WPILib VS Code** (the one the installer made, not regular VS Code) → **File → Open Folder** → pick the `SlipStreamSwerve` folder (the one with `build.gradle` in it).
4. Ctrl+Shift+P → **WPILib: Build Robot Code**. The first build needs internet and takes a few minutes while it downloads REVLib and Phoenix 6.
5. Firmware: update all SPARK Flexes to 2026 firmware in **REV Hardware Client**; update the CANcoders and Pigeon in **Phoenix Tuner X**.
6. Set CAN IDs to match `Constants.CanIds` (REV Hardware Client for SPARKs, Tuner X for CTRE). Leave the PDH at ID 1.

If your installed WPILib is older than 2026.2.1, either update WPILib or change the version number on line 3 of `build.gradle` to match yours.

## Test it in the simulator first (no robot needed)

The same code runs on the real robot and on your laptop. On the laptop it automatically swaps the motors, CANcoders and Pigeon for a physics model that uses the same PID gains, so you can drive a virtual robot before touching hardware.

**Start the simulator**
1. Plug in your Xbox controller **before** starting.
2. VS Code: Ctrl+Shift+P → **WPILib: Simulate Robot Code** → pick **Sim GUI** if it asks.
3. A "Robot Simulation" window opens. In it:
   - Drag your controller from **System Joysticks** onto **Joystick 0** in the **Joysticks** box.
   - In the **FMS** box, set **Alliance Station** to **Blue 1** (the simulator defaults to red, which flips which way "forward" goes).
   - In **Robot State**, click **Teleoperated**. The robot only moves when it's enabled, just like the real one.

**Watch it in AdvantageScope**
1. **File → Connect to Simulator.**
2. Click **+** → **2D Field**. In the left list, drag **Swerve → Pose** into the box at the bottom.
3. Click **+** → **Swerve**. Drag **Swerve → States** onto it (the actual wheels), then **Swerve → Setpoints** (what the code asked for), and **Swerve → Rotation** for the heading.
4. Drive with the controller. The robot moves on the field and the four wheels steer in the Swerve tab.

**What the simulator does and doesn't prove**
- It *does* catch logic mistakes: wrong field-relative directions, controls mapped wrong, wheels pointing the wrong way for a turn, crashes on startup, wildly wrong PID gains.
- It *can't* check anything physical: CANcoder offsets, motor inversions, CAN wiring, friction or battery sag. That's what the bring-up checklist below is for.
- `CAN Utilization %` reads 0 in the simulator; it's only meaningful on the robot.

## Bring-up checklist (robot ON BLOCKS, wheels off the ground)

1. **Enable once, check the dashboard.** Every `Swerve/<module>/CANcoder OK` and `Seeded` should be true, `Swerve/Gyro OK` true. Driver Station console should show no `[Swerve]` errors.
2. **CANcoder direction test.** Disabled, spin a wheel's steering **counter-clockwise viewed from above**. `Steer Abs (deg)` must increase. If not, flip `CANCODER_CCW_POSITIVE`.
3. **Steering direction test.** Same motion: `Steer Rel (deg)` must also increase. If not, flip `STEER_INVERTED`.
4. **CANcoder offsets.** Set all offsets to `0.0`, deploy. Point every wheel straight forward with **all bevel gears facing the robot's left**. Read `Swerve/<module>/Abs Raw (rot)` and put the **negative** of each value in `Constants.Modules`. Redeploy; all four `Steer Abs (deg)` should now read ~0.
5. **Drive direction.** Push the left stick forward gently. Any wheel spinning backwards → set that module's `driveInverted` to `true`.
6. **Gyro.** Rotate the robot CCW by hand: `Swerve/Heading (deg)` must increase. If not, fix the Pigeon mount pose.
7. Put it on the floor. Point the robot away from you, press **Start** to zero heading, drive.

## Tuning (with `TUNING_MODE = true`, values under `Tuning/` on the dashboard)

**Steer:** set Drive kP to 0 for now. Raise *Steer kP* until wheels snap to angle with a tiny overshoot, then back off ~20%. Add a small *Steer kD* only if they oscillate.

**Drive feedforward first:** *Drive kV* starts at the theoretical 12 V ÷ free speed. Drive at a steady speed and compare `Speed (mps)` to the commanded speed — raise kV if the wheels run slow, lower it if fast. *kS* is the voltage that just barely gets the wheel moving (0.1–0.25 V is typical).

**Then drive kP:** add a little until speed tracks well under load without buzzing.

Copy the final numbers into `Constants.Gains` and set `TUNING_MODE = false` for competition. Once steering and the gyro are verified, try `HEADING_HOLD_ENABLED = true` to stop the robot from drifting in rotation while translating.

## Heads-up about the 2027 season

This targets WPILib 2026 because you're on a roboRIO 2.0. FRC moves to Systemcore for 2027, and WPILib 2027 does not support the roboRIO (packages also rename from `edu.wpi.first` to `org.wpilib`). The swerve logic carries over, but you'll need to port it for the 2027 libraries.
