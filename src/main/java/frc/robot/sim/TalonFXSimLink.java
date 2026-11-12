package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Wires one simulated TalonFX to a WPILib physics object. Each step: {@link #motorVolts()} gives
 * the motor the battery's voltage and returns what the motor applies, which the physics takes
 * as input; then {@link #setMechanism} writes the physics' result back as rotor position and
 * velocity in rotor units. CTRE's documentation asks for both, so that the motor's own closed
 * loop and current limits behave as on the robot. Positions here are raw: the motor's own
 * {@code setPosition} offsets and sensor-to-mechanism ratio still apply on top.
 */
public final class TalonFXSimLink {
  private final TalonFXSimState state;
  private final double rotorPerMechanism;

  private TalonFXSimLink(TalonFX motor, double rotorPerMechanism, boolean clockwisePositive) {
    this.state = motor.getSimState();
    this.rotorPerMechanism = rotorPerMechanism;
    state.Orientation =
        clockwisePositive ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
    state.setMotorType(TalonFXSimState.MotorType.KrakenX44);
  }

  /**
   * A motor turning a mechanism through a reduction; the mechanism is in rotations.
   *
   * @param gearRatio rotor rotations per mechanism rotation (greater than one is a reduction)
   * @param clockwisePositive true when the motor's {@code Inverted} is {@code Clockwise_Positive}
   *     (or, for a follower, when it turns opposite to a leader that is not), so that positive
   *     mechanism motion is positive in the sim
   */
  public static TalonFXSimLink rotary(TalonFX motor, double gearRatio, boolean clockwisePositive) {
    return new TalonFXSimLink(motor, gearRatio, clockwisePositive);
  }

  /**
   * A motor driving a drum that winds a carriage; the mechanism is in metres.
   *
   * @param gearRatio rotor rotations per drum rotation
   * @param drumRadiusMeters the drum's effective radius: carriage travel per drum radian
   */
  public static TalonFXSimLink linear(
      TalonFX motor, double gearRatio, double drumRadiusMeters, boolean clockwisePositive) {
    return new TalonFXSimLink(motor, gearRatio / (2 * Math.PI * drumRadiusMeters), clockwisePositive);
  }

  /** Feeds the motor the simulated battery voltage and returns the voltage it applies. */
  public double motorVolts() {
    state.setSupplyVoltage(RobotController.getBatteryVoltage());
    return state.getMotorVoltage();
  }

  /** Writes the mechanism's position and velocity (rotations or metres, per second) as rotor units. */
  public void setMechanism(double position, double velocity) {
    state.setRawRotorPosition(position * rotorPerMechanism);
    state.setRotorVelocity(velocity * rotorPerMechanism);
  }

  /** The current this motor draws from its supply, as the motor's model computes it. */
  public double supplyCurrentAmps() {
    return state.getSupplyCurrent();
  }
}
