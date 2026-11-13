package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.forge.Diagnostics;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hardware.BenchWiring;
import frc.robot.hardware.Unit;
import frc.robot.subsystems.BenchStand;
import frc.robot.subsystems.Elevator;

/** The subsystems of this unit, built from its hardware map, and the controls bound to them. */
@Logged
public class RobotContainer {
  /**
   * The Driver Station port that Forge's virtual Xbox pad is read from. The harness plugs the pad
   * in once, at start-up, and it stays at rest except while a person has let Forge drive and holds
   * enable; set it to this port once in the Driver Station's USB tab, so robot code does not change
   * per run. Port 0, since this template reserves no port for a driver; a person's controller goes
   * on another port.
   */
  public static final int FORGE_PAD_PORT = 0;

  final Unit unit;
  final CommandXboxController forgePad = new CommandXboxController(FORGE_PAD_PORT);
  final CommandXboxController lampPad = new CommandXboxController(BenchWiring.LAMP_PAD_PORT);
  final Elevator elevator;
  final BenchStand bench;

  public RobotContainer(Unit unit) {
    this.unit = unit;
    elevator = new Elevator();
    bench = new BenchStand();
    configureBindings();
  }

  /** How long a stick dropout is bridged by the last good reading, in seconds. */
  static final double STICK_DROPOUT_SECONDS = 0.5;

  @NotLogged private final Diagnostics diag = new Diagnostics();
  private long stickCalls;
  private double lastStick = 0.0;
  private double lastStickTime = Double.NEGATIVE_INFINITY;

  /**
   * The stick for the elevator: forward is positive. The simulator hands the robot fresh joystick
   * data only on the cycle after each update and reports "no axes" in between, so a reading with
   * no axes is a dropout, not a stick at rest. A dropout keeps the last good value for
   * STICK_DROPOUT_SECONDS, then reads 0 (a pad that is really gone).
   */
  private double stickHeightAxis() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    stickCalls++;
    boolean fresh = edu.wpi.first.wpilibj.DriverStation.getStickAxisCount(FORGE_PAD_PORT) > 0;
    double read = fresh ? -forgePad.getLeftY() : Double.NaN;
    if (fresh) {
      lastStick = read;
      lastStickTime = now;
    } else if (now - lastStickTime > STICK_DROPOUT_SECONDS) {
      lastStick = 0.0;
    }
    diag.publishStick(stickCalls, fresh, read, lastStick, now - lastStickTime);
    return lastStick;
  }

  /** Read-backs on /Forge/Diag, once per robot cycle. */
  public void publishDiagnostics() {
    diag.publishRobot();
    diag.publishPad(FORGE_PAD_PORT, "Forge");
    diag.publishPad(BenchWiring.LAMP_PAD_PORT, "Lamp");
    elevator.publishDiagnostics(diag.table());
  }

  private void configureBindings() {
    // Forge drives through forgePad, which reads as an untouched controller unless Forge is
    // driving; bind the same commands to a driver's pad on another port.
    forgePad.x().onTrue(elevator.home());
    // The main stick sets the height: pushed forward (left Y negative) is up, released is the
    // bottom. The target moves toward it at a moderate rate. Jog and home take over while held.
    elevator.setDefaultCommand(elevator.followAxis(this::stickHeightAxis));
    forgePad.rightBumper().whileTrue(elevator.jog(() -> 1.0));
    forgePad.leftBumper().whileTrue(elevator.jog(() -> -1.0));

    // The bench lamp, unchanged except for its pad: right trigger sets the lamp brightness
    // (0 to 1); hold A for the automatic sweep instead.
    bench.setDefaultCommand(bench.joystickLamp(lampPad::getRightTriggerAxis));
    lampPad.a().whileTrue(bench.sweepLamp());
  }
}
