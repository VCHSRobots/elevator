package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A stand-in mechanism that exists only in simulation: a position loop on a simulated Kraken X60.
 * It drives no hardware, so it is safe on any robot. It shows the shape a real subsystem takes
 * (state as @Logged fields, commands as factories, a simulation model) and is what the scenario
 * test exercises. Replace it with the real subsystems.
 */
@Logged
public class ExampleSubsystem extends SubsystemBase {
  private static final double TOLERANCE_ROTATIONS = 0.02;

  private final PIDController pid = new PIDController(6.0, 0.0, 0.2);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.005, 10.0),
          DCMotor.getKrakenX60(1));

  double targetRotations;
  double positionRotations;
  double appliedVolts;

  /** Moves to a position and ends there. */
  public Command goTo(double rotations) {
    return runOnce(() -> targetRotations = rotations)
        .andThen(run(() -> {}).until(this::atTarget))
        .withName("ExampleGoTo");
  }

  public boolean atTarget() {
    return Math.abs(targetRotations - positionRotations) < TOLERANCE_ROTATIONS;
  }

  public double position() {
    return positionRotations;
  }

  @Override
  public void periodic() {
    appliedVolts =
        DriverStation.isEnabled()
            ? MathUtil.clamp(pid.calculate(positionRotations, targetRotations), -12.0, 12.0)
            : 0.0;
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    positionRotations = sim.getAngularPositionRotations();
  }
}
