package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.forge.ForgeAbort;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorConstants;
import org.junit.jupiter.api.Test;

/** The elevator in simulation, through the same Phoenix 6 code that runs on the robot. */
class ElevatorScenarioTest {
  private static final double TOLERANCE = 0.005; // the stand's travel is only 0.24 m
  private static final double MID = 0.12;

  @Test
  void itReachesAHeightAndNeverPassesTheTravel() {
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.0)) {
      scenario.run(0.1); // the bottom switch is pressed at rest: it zeroes the elevator
      assertTrue(elevator.zeroed());
      scenario.enable();

      CommandScheduler.getInstance().schedule(elevator.goTo(MID));
      scenario.run(8.0); // at 0.15 m/s the 0.12 m takes about 1 s
      assertEquals(MID, elevator.simHeightMeters(), TOLERANCE);
      assertEquals(MID, elevator.heightMeters(), TOLERANCE);

      // Asked past the top, it stops at the soft limit, inside the travel.
      double[] highest = {0.0};
      CommandScheduler.getInstance().schedule(elevator.goTo(ElevatorConstants.TRAVEL_METERS + 0.1));
      scenario.run(8.0, () -> highest[0] = Math.max(highest[0], elevator.simHeightMeters()));
      assertTrue(highest[0] <= Elevator.MAX_HEIGHT_METERS + TOLERANCE, "highest " + highest[0]);
      assertTrue(highest[0] < ElevatorConstants.TRAVEL_METERS, "it reached the hard stop");
      assertEquals(Elevator.MAX_HEIGHT_METERS, elevator.simHeightMeters(), TOLERANCE);
    }
  }

  @Test
  void theFrameStaysStillWhileOnlyTheCarriageRises() {
    var nt = NetworkTableInstance.getDefault();
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.0);
        var robotPose = nt.getStructTopic("/Forge/Elevator/RobotPose", Pose3d.struct).subscribe(new Pose3d());
        var components =
            nt.getStructArrayTopic("/Forge/Elevator/Components", Pose3d.struct)
                .subscribe(new Pose3d[0])) {
      scenario.run(0.1);
      scenario.enable();
      CommandScheduler.getInstance().schedule(elevator.goTo(MID));
      scenario.run(8.0);
      assertEquals(0.0, robotPose.get().getZ(), 1e-9);
      assertEquals(0.0, robotPose.get().getX(), 1e-9);
      assertEquals(
          ElevatorConstants.CARRIAGE_ORIGIN_AT_BOTTOM.getZ() + MID,
          components.get()[0].getZ(),
          TOLERANCE);
    }
  }

  @Test
  void theBottomSwitchZeroesIt() {
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.1)) {
      scenario.run(0.1);
      assertFalse(elevator.zeroed(), "it starts off the switch, so its height is unknown");
      scenario.enable();
      CommandScheduler.getInstance().schedule(elevator.goTo(0.2));
      scenario.run(0.5);
      // Disabled, the sim has no brake, so the carriage sags and settles back near where it
      // started; what matters is that goTo did not send it up to 0.2 m.
      assertTrue(elevator.simHeightMeters() <= 0.1 + TOLERANCE, "goTo is refused until homed");

      CommandScheduler.getInstance().schedule(elevator.home());
      scenario.run(12.0);
      assertTrue(elevator.bottomPressed());
      assertTrue(elevator.zeroed());
      assertEquals(0.0, elevator.heightMeters(), TOLERANCE);
      assertEquals(0.0, elevator.simHeightMeters(), TOLERANCE);
    }
  }

  @Test
  void itDoesNotMoveWhileDisabled() {
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.15)) {
      CommandScheduler.getInstance().schedule(elevator.home());
      scenario.run(1.0);
      // Disabled, the motors are neutral in brake; the sim has no brake, so gravity lowers it.
      // What matters is that nothing drove it: it never rose.
      assertTrue(elevator.simHeightMeters() <= 0.15 + 1e-6);
    }
  }

  @Test
  void forgesAbortFlagHoldsItWithZeroOutput() {
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.0);
        BooleanPublisher abort =
            NetworkTableInstance.getDefault().getBooleanTopic(ForgeAbort.TOPIC).publish()) {
      abort.set(false);
      scenario.run(0.1);
      scenario.enable();
      CommandScheduler.getInstance().schedule(elevator.goTo(0.2));
      scenario.run(1.0);
      double rising = elevator.simHeightMeters();
      assertTrue(rising > 0.02, "it started up: " + rising);

      abort.set(true); // a Forge script's limit tripped
      double[] highest = {0.0};
      scenario.run(2.0, () -> highest[0] = Math.max(highest[0], elevator.simHeightMeters()));
      assertTrue(elevator.aborted());
      assertTrue(highest[0] < 0.2 - TOLERANCE, "it did not go on to the target: " + highest[0]);
      assertEquals(0.0, elevator.appliedVolts(), 0.05, "zero output while the flag is set");

      abort.set(false); // a person clears it: it holds where it is, it does not resume
      double cleared = elevator.simHeightMeters();
      scenario.run(1.5);
      assertFalse(elevator.aborted());
      assertEquals(cleared, elevator.simHeightMeters(), 0.05);
    }
  }

  @Test
  void theStickSetsTheHeightAtAModerateRate() {
    var stick = new double[] {0.0};
    try (Scenario scenario = Scenario.start();
        Elevator elevator = new Elevator(0.0)) {
      elevator.setDefaultCommand(elevator.followAxis(() -> stick[0]));
      scenario.run(0.1);
      scenario.enable();
      scenario.run(1.0);
      assertEquals(0.0, elevator.simHeightMeters(), TOLERANCE, "stick at rest stays at the bottom");

      stick[0] = 1.0; // full forward: the top, reached at no more than STICK_RATE_MPS
      double[] fastest = {0.0};
      double[] last = {0.0};
      scenario.run(
          3.0,
          () -> {
            fastest[0] = Math.max(fastest[0], (elevator.simHeightMeters() - last[0]) / 0.02);
            last[0] = elevator.simHeightMeters();
          });
      assertEquals(Elevator.MAX_HEIGHT_METERS, elevator.simHeightMeters(), TOLERANCE);
      assertTrue(fastest[0] < ElevatorConstants.STICK_RATE_MPS * 1.5, "too fast: " + fastest[0]);

      stick[0] = 0.5;
      scenario.run(3.0);
      assertEquals(Elevator.MAX_HEIGHT_METERS / 2, elevator.simHeightMeters(), TOLERANCE);
      stick[0] = 0.0;
      scenario.run(3.0);
      assertEquals(0.0, elevator.simHeightMeters(), TOLERANCE);
    }
  }
}
