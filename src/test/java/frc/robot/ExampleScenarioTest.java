package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ExampleSubsystem;
import org.junit.jupiter.api.Test;

class ExampleScenarioTest {
  @Test
  void theMechanismReachesItsTargetWhenEnabled() {
    try (Scenario scenario = Scenario.start()) {
      ExampleSubsystem example = new ExampleSubsystem();
      scenario.enable();
      CommandScheduler.getInstance().schedule(example.goTo(1.0));
      scenario.run(3.0);
      assertTrue(example.atTarget(), "position " + example.position());
      assertEquals(1.0, example.position(), 0.05);
    }
  }

  @Test
  void theMechanismDoesNotMoveWhileDisabled() {
    try (Scenario scenario = Scenario.start()) {
      ExampleSubsystem example = new ExampleSubsystem();
      var command = example.goTo(1.0);
      CommandScheduler.getInstance().schedule(command);
      scenario.run(1.0);
      assertFalse(command.isScheduled(), "a command does not run while disabled");
      assertEquals(0.0, example.position(), 1e-9);
    }
  }
}
