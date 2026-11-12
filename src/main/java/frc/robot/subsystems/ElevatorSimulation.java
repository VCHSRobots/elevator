package frc.robot.subsystems;

import static frc.robot.subsystems.ElevatorConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.sim.MechanismSim;
import frc.robot.sim.TalonFXSimLink;

/**
 * The elevator's physics in simulation: WPILib's ElevatorSim with the real motors, ratio, drum,
 * carriage mass and travel, driven by the leader's simulated output, and the limit switches
 * pressed at either end of travel. Built by {@link Elevator} only in simulation.
 */
final class ElevatorSimulation implements MechanismSim {
  /** A switch reads pressed within this distance of its end of travel. */
  static final double SWITCH_REACH_METERS = 0.002;

  private final ElevatorSim physics;
  private final TalonFXSimLink leader;
  private final TalonFXSimLink follower;
  private final DIOSim bottom;
  private final DIOSim top;
  private final DoublePublisher heightOut;
  private final DoublePublisher currentOut;

  ElevatorSimulation(
      TalonFX leaderMotor,
      TalonFX followerMotor,
      DigitalInput bottomSwitch,
      DigitalInput topSwitch,
      double startMeters) {
    physics =
        new ElevatorSim(
            MOTORS, GEAR_RATIO, CARRIAGE_MASS_KG, DRUM_RADIUS_METERS, 0.0, TRAVEL_METERS, true, startMeters);
    leader = TalonFXSimLink.linear(leaderMotor, GEAR_RATIO, DRUM_RADIUS_METERS, LEADER_CLOCKWISE_POSITIVE);
    follower =
        TalonFXSimLink.linear(
            followerMotor, GEAR_RATIO, DRUM_RADIUS_METERS, LEADER_CLOCKWISE_POSITIVE ^ FOLLOWER_OPPOSED);
    bottom = new DIOSim(bottomSwitch);
    top = new DIOSim(topSwitch);
    var table = NetworkTableInstance.getDefault().getTable(Elevator.TABLE + "/Sim");
    heightOut = table.getDoubleTopic("HeightMeters").publish();
    currentOut = table.getDoubleTopic("CurrentAmps").publish();
    writeSensors();
  }

  @Override
  public void update(double dtSeconds) {
    double volts = leader.motorVolts();
    follower.motorVolts();
    physics.setInputVoltage(volts);
    physics.update(dtSeconds);
    writeSensors();
  }

  private void writeSensors() {
    double height = physics.getPositionMeters();
    double speed = physics.getVelocityMetersPerSecond();
    leader.setMechanism(height, speed);
    follower.setMechanism(height, speed);
    bottom.setValue((height <= SWITCH_REACH_METERS) != LIMIT_PRESSED_IS_LOW);
    top.setValue((height >= TRAVEL_METERS - SWITCH_REACH_METERS) != LIMIT_PRESSED_IS_LOW);
  }

  /** Where the carriage really is, metres. */
  double heightMeters() {
    return physics.getPositionMeters();
  }

  @Override
  public double currentDrawAmps() {
    return leader.supplyCurrentAmps() + follower.supplyCurrentAmps();
  }

  @Override
  public void publish() {
    heightOut.set(physics.getPositionMeters());
    currentOut.set(currentDrawAmps());
  }
}
