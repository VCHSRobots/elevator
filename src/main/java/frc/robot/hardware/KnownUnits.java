package frc.robot.hardware;

import java.util.List;

/** Every unit this code knows, one entry each, chosen by the controller's serial at startup. */
public final class KnownUnits {
  public static final Unit SIMULATION = new Unit("simulation", "simulation", true);

  /** The elevator stand's roboRIO. Serial seen by the harness and the program log on 2026-11-11. */
  public static final Unit ELEVATOR_STAND = new Unit("elevator-stand", "031be686", true);

  /** Add one line per physical unit: its name and its roboRIO's serial (RobotController). */
  public static final List<Unit> ALL = List.of(SIMULATION, ELEVATOR_STAND);

  private KnownUnits() {}

  /** The unit with this serial, or an unknown unit that names the serial. */
  public static Unit forSerial(String serial) {
    for (Unit unit : ALL) {
      if (unit.serial().equals(serial)) {
        return unit;
      }
    }
    return new Unit("unknown", serial, false);
  }
}
