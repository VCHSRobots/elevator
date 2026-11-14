# Notes

How this repository is organized and why. Forge reads this file before every change and keeps it current; so should anyone else who changes the code.

## What it is

A WPILib 2026 Java command-based robot program for a roboRIO, team 4415. It builds, simulates and deploys with the standard WPILib tools (VS Code, or `./gradlew` with the WPILib installation) and depends on nothing from Forge. Phoenix 6 (26.3.0) is in `vendordeps/` for the CTRE devices (Kraken X60 motors on TalonFX).

## The elevator stand

This repository (`VCHSRobots/elevator`) is the code for the elevator test stand. It follows the EPIC-Forge 2026 template, including its simulation layer. `KnownUnits` lists one unit: `elevator-stand`, roboRIO serial `031be686`. Nothing has been deployed from here.

`subsystems/Elevator` is the template's two-Kraken Motion Magic elevator (right motor CAN 10 leads, left CAN 11 follows `Opposed`), with its facts in `ElevatorConstants`, each commented with where it came from: two Kraken X44, 2.875 motor rotations per shaft turn, 4.0 in of chain per shaft turn (effective drum radius 4 in / 2 pi), 9.5 in of travel, 0.25 in soft margin, 20 A stator limit, speeds scaled so `kSlowFirst` (0.25) gives about 6 in/s cruise (raised from 1 in/s for stick control; a person must watch it on hardware first).

**Facts we do not have** (stand-ins in `ElevatorConstants`, marked UNKNOWN): the limit switch DIO channels (no switches are known to be wired; 8 and 9 are stand-ins, an unwired input reads "not pressed", so the elevator never zeroes and `goTo` refuses on the real stand), the carriage mass (2 kg stand-in: the sim's gravity load and kG are guesses), the real travel (the Thing says about 15 in, the stand's code says 9.5 in), the sprocket diameter and the 3D origin of the carriage. Gains are computed from the motor model, not tuned.

**Not carried over:** the stand's own repository (`VCHSRobots/EL-Test-Using-Forge-2026`) has a different elevator program (NetworkTables `TargetInches`, a ramped setpoint). This one is driven by Forge's pad (below), not by that topic.

### Bench wiring and `BenchStand` (temporary)

The stand is wired for a bench test, not the final elevator (`hardware/BenchWiring.java` holds the channels): a heartbeat LED on DIO 0 (330 ohm to ground), a Talon SR on PWM 0 (12 V) driving a lamp through 27 ohm / 5 W, and the Talon's output (ahead of the 27 ohm resistor) tapped on analog 0 through a 10k / 4.7k divider. `subsystems/BenchStand.java` blinks the LED at 5 Hz from `periodic()` (so also while disabled), sweeps the lamp 0 to full to 0 every 4 s as the default command (enabled only; never negative), and logs `lampVolts` (pin volts x 14.7 / 4.7, 64-sample hardware average plus a 10-loop moving average because the Talon chops its output). The old Elevator-EASDL-Test-2026 code ran on the same hardware. Remove `BenchStand` and `BenchWiring` when the real wiring replaces this. Not yet deployed or run on the stand.

## Layout

- `src/main/java/frc/robot/Main.java` starts the robot program, or the idle program when built with `-Pprogram=idle`.
- `Robot.java` runs the command scheduler and binds Epilogue logging. It holds nothing else.
- `RobotContainer.java` builds the subsystems from this unit's hardware map and binds the controls.
- `IdleRobot.java` is the idle program: it reports its identity and constructs no motor controller, so enabling it moves nothing. It is left on the robot between sessions.
- `forge/Identity.java` starts the WPILOG log (`DataLogManager`), reads the controller's serial, picks the unit, and publishes the identity (below).
- `hardware/` holds the hardware map: `Unit` is what differs between two units running the same code (CAN IDs, inversions, offsets), and `KnownUnits` lists every physical unit by its roboRIO's serial. An unknown serial gives an unknown unit and a warning on the Driver Station.
- `subsystems/` holds one class per subsystem, each annotated `@Logged` so Epilogue logs its fields, with its facts in a constants file beside it and its physics in a sim class beside it: `Elevator`, `ElevatorConstants`, `ElevatorSimulation`, plus the temporary `BenchStand`.
- `sim/` is the simulation support: `MechanismSim` (what every subsystem's sim implements), `SimRegistry` (stepped from `Robot.simulationPeriodic()`, which also sags the battery by the current drawn), and `TalonFXSimLink` (wires a simulated Kraken, here X44, to WPILib physics).
- `forge/ForgeAbort` reads `/Forge/Abort`; `Elevator` commands zero output and holds while it is true, and holds where it is once it clears.
- `assets/Robot_Forge/` is the robot's model for AdvantageScope (its README says how to make and show it); `model.glb`, `model_0.glb` and `config.json` are committed. The carriage's calibration along the rails is still off. Lessons from the first simulation sessions are in `docs/sim-and-forge-lessons.md`: read it before simulating, driving a stick in the simulator or changing the 3D model.
- `src/test/java/frc/robot/` holds the scenario tests: `Scenario` enables the simulated Driver Station, runs the scheduler in 20 ms steps of simulated time, and the test asserts on the subsystems. Nothing here touches a real robot.

## Build identity

The Gradle task `writeBuildConstants` writes `build/generated/sources/forge/java/frc/robot/BuildConstants.java` before every compile: repository, branch, commit, whether the tree was dirty, build time, builder (`FORGE_BUILDER`, else the user name) and program (`robot` or `idle`). At startup the program publishes them to NetworkTables (NT) under `/Forge/Build/` (`Repository`, `Branch`, `Commit`, `Dirty`, `BuiltAt`, `Builder`, `Program`), with `/Forge/Serial` and `/Forge/Unit`, and writes the same as one JSON entry, `/Forge/Identity`, into the WPILOG file. The Forge harness reads these to say which build ran in each run. Built outside a Git checkout, the Git fields are empty.

## Commands

- `./gradlew build` compiles and runs the scenario tests. `./gradlew test` runs only the tests.
- `./gradlew simulateJava` runs the program in simulation with the GUI; add `-Pheadless` for none, and `-Pwebsockets` for the HAL WebSocket server (`ws://localhost:3300/wpilibws`), through which the harness enables the simulator and feeds its joysticks. The `addWebsocketsServer` line in `build.gradle` is what lets the harness act as the simulator's Driver Station; without it "Run in simulation" never comes up.
- `./gradlew deploy` deploys to the roboRIO at 10.44.15.2 (or USB). Add `-Pprogram=idle` to build or deploy the idle program.
- After one online build, `--offline` works, as at competition.

## Conventions

- Each subsystem keeps its state in `@Logged` fields and offers commands as factory methods.
- A thing's facts (CAN IDs, DIO ports, inversions, gear ratio, drum radius, mass, travel, limits) go in the subsystem's constants file with a comment where each came from; mark a fact nobody has told us UNKNOWN rather than guessing. A fact that differs between two units running the same code moves into `Unit`.
- One code path: `RobotBase.isSimulation()` decides only whether a subsystem builds its sim.
- Every subsystem comes with a sim that implements `MechanismSim` and registers with `SimRegistry` in its constructor when in simulation (Phoenix sim state through `TalonFXSimLink` to a WPILib physics class, limit switches through `DIOSim`, its current draw for the battery). When a subsystem changes, its sim changes in the same commit.
- Every subsystem publishes for AdvantageScope under `/Forge/<Subsystem>/`: a `Mechanism2d` at `Mechanism` and robot-relative `Pose3d` structs at `Components`, in the order of `assets/Robot_Forge/config.json`. The Elevator also publishes `/Forge/Elevator/RobotPose`, always the origin: put it on AdvantageScope's Robot row and add `Components` as a child of that row with type Component. Never put `Components` or a height on the Robot row itself, or the whole frame rises with the carriage.
- Every subsystem has a scenario test before it runs on hardware: it reaches its setpoint, never passes its limits, its switches stop and zero it. `Scenario` calls `SimRegistry.clear()` at start and `SimRegistry.update(STEP_SECONDS)` each step, paced in wall time because Phoenix 6's simulated devices run on the wall clock (the tests take about 40 s).
- The first physical runs use `kSlowFirst` (0.25): it scales every speed, acceleration and jog voltage. Raise it only after a person has watched the full travel and said so. Soft and current limits stay on always.
- Forge's virtual pad is read from `RobotContainer.FORGE_PAD_PORT` (port 0): the left stick sets the height (forward = up, released = bottom; `Elevator.followAxis`, the default command, moves the target at `STICK_RATE_MPS` = 0.12 m/s), X homes, bumpers jog. A person's controller goes on another port.
- Signals are logged through Epilogue and `DataLogManager`, so every run yields comparable WPILOG files.

## The WPILib year

This is the 2026 season: GradleRIO 2026.2.1 (`build.gradle`), `frcYear = '2026'` (`settings.gradle`, which also names the install folder `~/wpilib/2026` or `C:\Users\Public\wpilib\2026`), `projectYear` in `.wpilib/wpilib_preferences.json`, and the `frcYear` in both vendordeps. The roboRIO image, the Driver Station and WPILib must be the same season. To move to 2027:

1. Install that year's WPILib and flash the roboRIO. Set `WPILIB_HOME` for the harness, or change `default_wpilib_home` in `harness/src/forge_harness/config.py`.
2. Set the GradleRIO version, `frcYear` and `projectYear` to the new year. Read the version from the Gradle plugin portal (`edu.wpi.first.GradleRIO`).
3. Replace the vendordeps: `WPILibNewCommands.json` from the `allwpilib` tag of that release, and CTRE's `Phoenix6-frc<year>-latest.json` from `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/`. Delete the old Phoenix file. GradleRIO refuses a vendordep whose year is wrong, which is how you find any you missed.
4. Run `./gradlew build`, then `--offline`. Fix what the new WPILib or Epilogue changed.

### Joystick lamp control (temporary, bench wiring)
- `RobotContainer` binds a person's Xbox controller on port 1 (`BenchWiring.LAMP_PAD_PORT`; it was port 0 until Forge's pad took that): the **right trigger** sets the lamp brightness (0 to 1) and is the `BenchStand` default command. Holding **A** runs the old automatic sweep instead.
- `BenchStand` is now `AutoCloseable` so tests can build several; `joystickLamp(DoubleSupplier)` clamps to 0..`MAX_LAMP`.
