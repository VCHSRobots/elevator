# Simulation and Forge: what we learned

Written after the first long simulation session on the elevator stand. "Observed" means a log, a screenshot or a test showed it; "inferred" means we think so but did not prove it.

## What was accomplished

1. **Simulation layer** (PR #5): the elevator runs in simulation with simulated Krakens, physics and limit switches.
2. **3D pose publishing** (PR #6): `/Forge/Elevator/RobotPose` and `/Forge/Elevator/Components` for AdvantageScope.
3. **Stick controls height** (PR #7): the left stick of the pad on port 0 sets the carriage height, with the last good stick value held for 0.5 s over dropouts, plus `/Forge/Diag` read-backs.
4. **The 3D model shows in AdvantageScope** (`assets/Robot_Forge/`, committed with this note).
5. **A real controller drives the simulation** (the Logitech Extreme 3D), after the joystick-assignment problem below was found.

Not done: nothing is deployed, the real pad has not been tested on the stand, pulling the stick back from a raised height has not been tested, and the carriage calibration along the rails is still off.

## Simulator joystick (the biggest time sink)

- Observed: the simulated robot sees nothing from a real controller until someone drags it in the simulator window. In the **System Joysticks** box, drag "Logitech Extreme 3D" onto the **0:** slot. While the slot reads "(null)", every axis is `nan`, the axis count is 0 and the carriage never moves.
- Forge cannot do the drag: its screen tools click, type and scroll but cannot hold the mouse button. A person at the station does it once per simulator start.
- Observed: a `control: script` simulation gave an empty pad. Inferred: the script owns the simulator's joystick, so a real controller is ignored. For a real controller use `control: controller`.
- Not tried: saving the assignment in `simgui-ds.json` (the file is git-ignored). Worth trying so the drag is remembered.
- Extreme 3D: 4 axes and 12 buttons in the simulator. At rest axes read about `[0, 0, 0, 0.83]`; axis 3 is the throttle slider. Axis 1 is probably forward/back (forward negative), but no clean forward-then-back trace was recorded.
- The "no controller on port 1" warnings are expected: the code also reads a second pad (the lamp control).
- Observed: the simulator hands the robot stick data only part of the time. PR #7 holds the last good value for 0.5 s. Inferred: a real Driver Station sends every 20 ms and will not show this.
- Forge's virtual pad (`FORGE_PAD_PORT`, port 0) is read by the code. A person's pad on port 0 in simulation uses the same path.

## Working with the station harness

- The harness shell on Windows is `cmd`, not PowerShell. Chain commands with `&`.
- File tools only reach the Forge workspace (`C:\Users\<name>\forge-workspace`). To get a file from elsewhere, copy it into the workspace with the shell first.
- `read_file` returns binary files as base64 in JSON. The result is capped at about 2 MB, so split large files into pieces (for example with `python` on the station) and rejoin them; check the glTF header length against the total.
- Very large NetworkTables reads were followed by the station going offline (a guess at the cause). Read small, or pull the robot's log afterwards instead of polling by hand.
- After any station disconnect, read `harness_log` before guessing. The station came back in about a second.
- A restart of Forge pauses a running simulation, disabled, for up to two minutes; enable it again and carry on.
- Warm the Gradle cache with the `classes` task before a simulation card.
- Do not trust a hand-polled value over the robot's WPILOG. Several wrong claims in the session came from polling.

## Scripts versus driving the UI

- Use a **script** for anything timed or measured: it can enable the simulator, set values and record a CSV with timestamps every 0.05 s. It cannot drag or click in a GUI.
- Use **screen control** only when there is no file or command for the job (a dialog). It is slow, and it ends the moment a person touches the keyboard or mouse.
- Best pattern: a person does the one manual step (the drag), Forge enables the simulator, reads results from the log afterwards.
- Check a claim against a log or a screenshot before stating it.

## 3D model in AdvantageScope

- Make the model as the README in `assets/` says: STEP from CAD, then `.glb` through CAD Assistant with "Merge faces within the same part".
- Observed: CAD Assistant's `.glb` files were really JSON pointing at separate `.bin` files, and AdvantageScope would not load them. Repacking each pair into one self-contained binary `.glb` fixed it.
- In AdvantageScope choose Help, Use Custom Assets Folder, and pick the repository's `assets` folder. The robot named "Forge" then appears in the 3D Field.
- **Pose wiring:** put `/Forge/Elevator/RobotPose` (always the origin) on the **Robot** row and `/Forge/Elevator/Components` on a **Component** row. Observed: with `Components` on the Robot row the whole frame rose with the carriage.
- `config.json` holds the body rotations and offset and each component's `zeroedRotations` and `zeroedPosition`. It is committed here; AdvantageScope reads the copy in the folder you pointed it at, so copy changes there.
- The carriage's position along the rails is still off; adjust `zeroedPosition` and `CARRIAGE_ORIGIN_AT_BOTTOM` in `ElevatorConstants.java` together.

## Facts still unknown

- Limit-switch DIO channels (8 and 9 are stand-ins).
- Carriage mass (2 kg assumed).
- True travel (9.5 in in code, about 15 in on the Thing).
