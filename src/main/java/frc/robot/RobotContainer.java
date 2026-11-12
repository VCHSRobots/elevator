package frc.robot;

import edu.wpi.first.epilogue.Logged;
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

  private void configureBindings() {
    // Forge drives through forgePad, which reads as an untouched controller unless Forge is
    // driving; bind the same commands to a driver's pad on another port.
    forgePad.x().onTrue(elevator.home());
    forgePad.a().onTrue(elevator.goTo(0.0));
    forgePad.b().onTrue(elevator.goTo(Elevator.MAX_HEIGHT_METERS / 2));
    forgePad.y().onTrue(elevator.goTo(Elevator.MAX_HEIGHT_METERS));
    forgePad.rightBumper().whileTrue(elevator.jog(() -> 1.0));
    forgePad.leftBumper().whileTrue(elevator.jog(() -> -1.0));

    // The bench lamp, unchanged except for its pad: right trigger sets the lamp brightness
    // (0 to 1); hold A for the automatic sweep instead.
    bench.setDefaultCommand(bench.joystickLamp(lampPad::getRightTriggerAxis));
    lampPad.a().whileTrue(bench.sweepLamp());
  }
}
