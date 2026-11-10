# Notes

How this repository is organized and why. Forge reads this file before every change and keeps it current; so should anyone else who changes the code.

## What it is

A WPILib 2026 Java command-based robot program for a roboRIO, team 4415. It builds, simulates and deploys with the standard WPILib tools (VS Code, or `./gradlew` with the WPILib installation) and depends on nothing from Forge. Phoenix 6 (26.3.0) is in `vendordeps/` for the CTRE devices (Kraken X60 motors on TalonFX).

## The elevator stand

This repository (`VCHSRobots/elevator`) is the code for the elevator test stand. It started as the unchanged EPIC-Forge 2026 template. `KnownUnits` lists one unit so far: `elevator-stand`, roboRIO serial `03320607` (from the Test Controller's records; confirm it against `/Forge/Serial` on the first boot). `ExampleSubsystem` is still the template's placeholder: the elevator subsystem, its CAN IDs and its `Unit` constants are not written yet. Nothing has been deployed from here.

### Bench wiring and `BenchStand` (temporary)

The stand is wired for a bench test, not the final elevator (`hardware/BenchWiring.java` holds the channels): a heartbeat LED on DIO 0 (330 ohm to ground), a Talon SR on PWM 0 (12 V) driving a lamp through 27 ohm / 5 W, and the Talon's output (ahead of the 27 ohm resistor) tapped on analog 0 through a 10k / 4.7k divider. `subsystems/BenchStand.java` blinks the LED at 5 Hz from `periodic()` (so also while disabled), sweeps the lamp 0 to full to 0 every 4 s as the default command (enabled only; never negative), and logs `lampVolts` (pin volts x 14.7 / 4.7, 64-sample hardware average plus a 10-loop moving average because the Talon chops its output). The old Elevator-EASDL-Test-2026 code ran on the same hardware. Remove `BenchStand` and `BenchWiring` when the real wiring replaces this. Not yet deployed or run on the stand.

## Layout

- `src/main/java/frc/robot/Main.java` starts the robot program, or the idle program when built with `-Pprogram=idle`.
- `Robot.java` runs the command scheduler and binds Epilogue logging. It holds nothing else.
- `RobotContainer.java` builds the subsystems from this unit's hardware map and binds the controls.
- `IdleRobot.java` is the idle program: it reports its identity and constructs no motor controller, so enabling it moves nothing. It is left on the robot between sessions.
- `forge/Identity.java` starts the WPILOG log (`DataLogManager`), reads the controller's serial, picks the unit, and publishes the identity (below).
- `hardware/` holds the hardware map: `Unit` is what differs between two units running the same code (CAN IDs, inversions, offsets), and `KnownUnits` lists every physical unit by its roboRIO's serial. An unknown serial gives an unknown unit and a warning on the Driver Station.
- `subsystems/` holds one class per subsystem, each annotated `@Logged` so Epilogue logs its fields. `ExampleSubsystem` exists only in simulation and drives no hardware; replace it.
- `src/test/java/frc/robot/` holds the scenario tests: `Scenario` enables the simulated Driver Station, runs the scheduler in 20 ms steps of simulated time, and the test asserts on the subsystems. Nothing here touches a real robot.

## Build identity

The Gradle task `writeBuildConstants` writes `build/generated/sources/forge/java/frc/robot/BuildConstants.java` before every compile: repository, branch, commit, whether the tree was dirty, build time, builder (`FORGE_BUILDER`, else the user name) and program (`robot` or `idle`). At startup the program publishes them to NetworkTables (NT) under `/Forge/Build/` (`Repository`, `Branch`, `Commit`, `Dirty`, `BuiltAt`, `Builder`, `Program`), with `/Forge/Serial` and `/Forge/Unit`, and writes the same as one JSON entry, `/Forge/Identity`, into the WPILOG file. The Forge harness reads these to say which build ran in each run. Built outside a Git checkout, the Git fields are empty.

## Commands

- `./gradlew build` compiles and runs the scenario tests. `./gradlew test` runs only the tests.
- `./gradlew simulateJava` runs the program in simulation with the GUI; add `-Pheadless` for none.
- `./gradlew deploy` deploys to the roboRIO at 10.44.15.2 (or USB). Add `-Pprogram=idle` to build or deploy the idle program.
- After one online build, `--offline` works, as at competition.

## Conventions

- Each subsystem keeps its state in `@Logged` fields and offers commands as factory methods; constants a unit can differ in go in `Unit`, never in the subsystem.
- Every subsystem gets a simulation model in `simulationPeriodic` and at least one scenario test before it runs on hardware.
- Signals are logged through Epilogue and `DataLogManager`, so every run yields comparable WPILOG files.

## The WPILib year

This is the 2026 season: GradleRIO 2026.2.1 (`build.gradle`), `frcYear = '2026'` (`settings.gradle`, which also names the install folder `~/wpilib/2026` or `C:\Users\Public\wpilib\2026`), `projectYear` in `.wpilib/wpilib_preferences.json`, and the `frcYear` in both vendordeps. The roboRIO image, the Driver Station and WPILib must be the same season. To move to 2027:

1. Install that year's WPILib and flash the roboRIO. Set `WPILIB_HOME` for the harness, or change `default_wpilib_home` in `harness/src/forge_harness/config.py`.
2. Set the GradleRIO version, `frcYear` and `projectYear` to the new year. Read the version from the Gradle plugin portal (`edu.wpi.first.GradleRIO`).
3. Replace the vendordeps: `WPILibNewCommands.json` from the `allwpilib` tag of that release, and CTRE's `Phoenix6-frc<year>-latest.json` from `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/`. Delete the old Phoenix file. GradleRIO refuses a vendordep whose year is wrong, which is how you find any you missed.
4. Run `./gradlew build`, then `--offline`. Fix what the new WPILib or Epilogue changed.

### Joystick lamp control (temporary, bench wiring)
- `RobotContainer` binds an Xbox controller on port 0: the **right trigger** sets the lamp brightness (0 to 1) and is the `BenchStand` default command. Holding **A** runs the old automatic sweep instead.
- `BenchStand` is now `AutoCloseable` so tests can build several; `joystickLamp(DoubleSupplier)` clamps to 0..`MAX_LAMP`.
