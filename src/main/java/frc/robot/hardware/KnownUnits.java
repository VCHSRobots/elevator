package frc.robot.hardware;

import java.util.List;

/** Every unit this code knows, one entry each, chosen by the controller's serial at startup. */
public final class KnownUnits {
  public static final Unit SIMULATION = new Unit("simulation", "simulation", true);

  /** Add one line per physical unit: its name and its roboRIO's serial (RobotController). */
  public static final List<Unit> ALL = List.of(SIMULATION);

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
