package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.BenchWiring;

/**
 * The elevator stand's bench test: a heartbeat LED that blinks at 5 Hz whenever the program runs
 * (enabled or not), a lamp on a Talon SR whose brightness sweeps up and down while enabled, and a
 * readback of the voltage at the heartbeat LED's anode.
 */
@Logged
public class BenchStand extends SubsystemBase {
  /** Blink rate of the heartbeat LED. */
  public static final double HEARTBEAT_HZ = 5.0;

  /** Seconds for the lamp to go from off to full and back to off. */
  public static final double SWEEP_PERIOD_S = 4.0;

  /** Lamp command limit. The Talon is only ever commanded 0 to this, never negative. */
  public static final double MAX_LAMP = 1.0;

  private final Talon lamp = new Talon(BenchWiring.PWM_LAMP);
  private final DigitalOutput heartbeatLed = new DigitalOutput(BenchWiring.DIO_HEARTBEAT_LED);
  private final AnalogInput ledSense = new AnalogInput(BenchWiring.ANALOG_LED_SENSE);

  boolean heartbeatOn;
  double lampCommand;
  double ledAnodeVolts;

  /** True for the first half of each blink period, false for the second. */
  public static boolean heartbeatLevel(double seconds) {
    double halfPeriod = 0.5 / HEARTBEAT_HZ;
    return ((long) Math.floor(seconds / halfPeriod)) % 2 == 0;
  }

  /** A triangle wave from 0 up to {@code MAX_LAMP} and back, one cycle per sweep period. */
  public static double sweep(double seconds) {
    double phase = (seconds / SWEEP_PERIOD_S) % 1.0;
    if (phase < 0) {
      phase += 1.0;
    }
    double triangle = phase < 0.5 ? 2.0 * phase : 2.0 * (1.0 - phase);
    return triangle * MAX_LAMP;
  }

  public BenchStand() {
    setDefaultCommand(sweepLamp());
  }

  /** Sweeps the lamp's brightness up and down until interrupted; leaves it off at the end. */
  public Command sweepLamp() {
    return runEnd(() -> setLamp(sweep(Timer.getFPGATimestamp())), () -> setLamp(0.0))
        .withName("SweepLamp");
  }

  private void setLamp(double value) {
    lampCommand = Math.max(0.0, Math.min(MAX_LAMP, value));
    lamp.set(lampCommand);
  }

  public double lampCommand() {
    return lampCommand;
  }

  public boolean heartbeatOn() {
    return heartbeatOn;
  }

  public double ledAnodeVolts() {
    return ledAnodeVolts;
  }

  @Override
  public void periodic() {
    heartbeatOn = heartbeatLevel(Timer.getFPGATimestamp());
    heartbeatLed.set(heartbeatOn);
    ledAnodeVolts = BenchWiring.anodeVolts(ledSense.getVoltage());
  }
}
