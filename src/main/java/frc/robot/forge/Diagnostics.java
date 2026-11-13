package frc.robot.forge;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Read-back values on NetworkTables under {@code /Forge/Diag}, so a run's log shows where a signal
 * is lost between the Driver Station and the carriage: what the DS layer reports for the pad, what
 * the stick code used, what the commands did and what the elevator decided. Everything is
 * published every robot cycle; {@code Cycle} counts the cycles so a gap shows.
 */
public final class Diagnostics {
  public static final String TABLE = "/Forge/Diag";

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE);
  private long cycle;
  private long commandStarts;
  private long commandEnds;

  public Diagnostics() {
    var scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(
        c -> {
          commandStarts++;
          table.getEntry("Cmd/LastStarted").setString(c.getName());
          table.getEntry("Cmd/Starts").setInteger(commandStarts);
        });
    scheduler.onCommandFinish(
        c -> {
          commandEnds++;
          table.getEntry("Cmd/LastFinished").setString(c.getName());
          table.getEntry("Cmd/Ends").setInteger(commandEnds);
        });
    scheduler.onCommandInterrupt(
        c -> {
          commandEnds++;
          table.getEntry("Cmd/LastInterrupted").setString(c.getName());
          table.getEntry("Cmd/Ends").setInteger(commandEnds);
        });
  }

  /** The Driver Station layer's view of one port: what the robot code can possibly see. */
  public void publishPad(int port, String name) {
    String p = "Pad/" + name + "/";
    table.getEntry(p + "AxisCount").setInteger(DriverStation.getStickAxisCount(port));
    table.getEntry(p + "ButtonCount").setInteger(DriverStation.getStickButtonCount(port));
    table.getEntry(p + "PovCount").setInteger(DriverStation.getStickPOVCount(port));
    table.getEntry(p + "Connected").setBoolean(DriverStation.isJoystickConnected(port));
    table
        .getEntry(p + "RawAxis1")
        .setDouble(
            DriverStation.getStickAxisCount(port) > 1
                ? DriverStation.getStickAxis(port, 1)
                : Double.NaN);
    table.getEntry(p + "Buttons").setInteger(DriverStation.getStickButtons(port));
  }

  /** The stick code's own steps: how often it ran, what it read, what it handed on. */
  public void publishStick(
      long calls, boolean fresh, double read, double used, double secondsSinceGood) {
    table.getEntry("Stick/Calls").setInteger(calls);
    table.getEntry("Stick/Fresh").setBoolean(fresh);
    table.getEntry("Stick/Read").setDouble(read);
    table.getEntry("Stick/Used").setDouble(used);
    table.getEntry("Stick/SecondsSinceGood").setDouble(secondsSinceGood);
  }

  /** Robot state and the cycle counter; call once per robot cycle. */
  public void publishRobot() {
    cycle++;
    table.getEntry("Cycle").setInteger(cycle);
    table.getEntry("DS/Enabled").setBoolean(DriverStation.isEnabled());
    table.getEntry("DS/Teleop").setBoolean(DriverStation.isTeleopEnabled());
    table.getEntry("DS/Attached").setBoolean(DriverStation.isDSAttached());
    table.getEntry("DS/MatchTime").setDouble(DriverStation.getMatchTime());
  }

  public NetworkTable table() {
    return table;
  }
}
