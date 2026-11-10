package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hardware.Unit;
import frc.robot.subsystems.BenchStand;
import frc.robot.subsystems.ExampleSubsystem;

/** The subsystems of this unit, built from its hardware map, and the controls bound to them. */
@Logged
public class RobotContainer {
  final Unit unit;
  final ExampleSubsystem example;
  final BenchStand bench;
  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer(Unit unit) {
    this.unit = unit;
    example = new ExampleSubsystem();
    bench = new BenchStand();
    configureBindings();
  }

  private void configureBindings() {
    // Right trigger sets the lamp brightness (0 to 1). Hold A for the automatic sweep instead.
    bench.setDefaultCommand(bench.joystickLamp(driver::getRightTriggerAxis));
    driver.a().whileTrue(bench.sweepLamp());
  }
}
