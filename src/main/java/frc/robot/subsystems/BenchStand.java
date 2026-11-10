package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.BenchWiring;

/**
 * The elevator stand's bench test: a heartbeat LED that blinks at 5 Hz whenever the program runs
 * (enabled or not), a lamp on a Talon SR whose brightness sweeps up and down while enabled, and a
 * readback of the voltage the Talon delivers (averaged, because the Talon chops its output).
 */
@Logged
public class BenchStand extends SubsystemBase implements AutoCloseable {
  /** Blink rate of the heartbeat LED. */
  public static final double HEARTBEAT_HZ = 5.0;

  /** Seconds for the lamp to go from off to full and back to off. */
  public static final double SWEEP_PERIOD_S = 4.0;

  /** Lamp command limit. The Talon is only ever commanded 0 to this, never negative. */
  public static final double MAX_LAMP = 1.0;

  private final Talon lamp = new Talon(BenchWiring.PWM_LAMP);
  private final DigitalOutput heartbeatLed = new DigitalOutput(BenchWiring.DIO_HEARTBEAT_LED);
  private final AnalogInput lampSense = new AnalogInput(BenchWiring.ANALOG_LAMP_SENSE);

  /** Software average over this many 20 ms loops, on top of the hardware averaging. */
  private static final int AVERAGE_LOOPS = 10;

  private final LinearFilter lampVoltsFilter = LinearFilter.movingAverage(AVERAGE_LOOPS);

  boolean heartbeatOn;
  double lampCommand;
  double lampVolts;

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
    lampSense.setAverageBits(6); // 64 samples per reading smooths the Talon's chopping
    setDefaultCommand(sweepLamp()); // RobotContainer replaces this with the joystick
  }

  /** Sets the lamp's brightness from an input (0 to 1, such as a trigger) until interrupted. */
  public Command joystickLamp(DoubleSupplier brightness) {
    return runEnd(() -> setLamp(brightness.getAsDouble() * MAX_LAMP), () -> setLamp(0.0))
        .withName("JoystickLamp");
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

  public double lampVolts() {
    return lampVolts;
  }

  /** Releases the channels (the tests build several of these in one run). */
  @Override
  public void close() {
    Command running = getCurrentCommand();
    if (running != null) {
      running.cancel();
    }
    CommandScheduler.getInstance().unregisterSubsystem(this);
    lamp.close();
    heartbeatLed.close();
    lampSense.close();
  }

  @Override
  public void periodic() {
    heartbeatOn = heartbeatLevel(Timer.getFPGATimestamp());
    heartbeatLed.set(heartbeatOn);
    lampVolts = lampVoltsFilter.calculate(BenchWiring.lampVolts(lampSense.getAverageVoltage()));
  }
}
