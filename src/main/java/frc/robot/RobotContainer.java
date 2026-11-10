package frc.robot;

import edu.wpi.first.epilogue.Logged;
import frc.robot.hardware.Unit;
import frc.robot.subsystems.ExampleSubsystem;

/** The subsystems of this unit, built from its hardware map, and the controls bound to them. */
@Logged
public class RobotContainer {
  final Unit unit;
  final ExampleSubsystem example;

  public RobotContainer(Unit unit) {
    this.unit = unit;
    example = new ExampleSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    // Bind driver controls here, for example with CommandXboxController.
  }
}
