package frc.robot.hardware;

/**
 * The elevator stand's temporary bench wiring (not the final wiring). Replace this when the real
 * elevator is wired.
 *
 * <ul>
 *   <li>DIO 0: heartbeat LED, anode on the pin, cathode through 330 ohm to ground.
 *   <li>PWM 0: Talon SR (12 V supply) driving a lamp through a 27 ohm, 5 W resistor.
 *   <li>Analog 0: the LED's anode through a divider, 10k from the LED and 4.7k to ground.
 * </ul>
 */
public final class BenchWiring {
  public static final int PWM_LAMP = 0;
  public static final int DIO_HEARTBEAT_LED = 0;
  public static final int ANALOG_LED_SENSE = 0;

  public static final double DIVIDER_UPPER_OHMS = 10_000.0;
  public static final double DIVIDER_LOWER_OHMS = 4_700.0;

  private BenchWiring() {}

  /** Converts the volts at the analog pin to the volts at the LED's anode. */
  public static double anodeVolts(double pinVolts) {
    return pinVolts * (DIVIDER_UPPER_OHMS + DIVIDER_LOWER_OHMS) / DIVIDER_LOWER_OHMS;
  }
}
