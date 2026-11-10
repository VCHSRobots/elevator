package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import frc.robot.hardware.BenchWiring;
import frc.robot.subsystems.BenchStand;
import org.junit.jupiter.api.Test;

class BenchStandTest {
  @Test
  void heartbeatBlinksAtFiveHertz() {
    int toggles = 0;
    boolean last = BenchStand.heartbeatLevel(0.0);
    for (int i = 1; i <= 1000; i++) { // 10 s in 10 ms steps
      boolean now = BenchStand.heartbeatLevel(i * 0.01 + 1e-6);
      if (now != last) {
        toggles++;
      }
      last = now;
    }
    assertEquals(100, toggles, 1); // 5 Hz = 10 toggles per second
    assertTrue(BenchStand.heartbeatLevel(0.05));
    assertFalse(BenchStand.heartbeatLevel(0.15));
  }

  @Test
  void sweepStaysInRangeAndPeaksMidPeriod() {
    for (double t = 0; t < 20; t += 0.013) {
      double v = BenchStand.sweep(t);
      assertTrue(v >= 0.0 && v <= BenchStand.MAX_LAMP, "sweep " + v + " at " + t);
    }
    assertEquals(0.0, BenchStand.sweep(0.0), 1e-9);
    assertEquals(BenchStand.MAX_LAMP, BenchStand.sweep(BenchStand.SWEEP_PERIOD_S / 2), 1e-9);
    assertEquals(0.0, BenchStand.sweep(BenchStand.SWEEP_PERIOD_S), 1e-9);
  }

  @Test
  void dividerReadsTheTalonOutput() {
    assertEquals(2.0, BenchWiring.lampVolts(2.0 * 4.7 / 14.7), 1e-9);
  }

  @Test
  void lampSweepsWhenEnabledAndHeartbeatRunsWhenDisabled() {
    try (Scenario scenario = Scenario.start()) {
      BenchStand bench = new BenchStand();
      AnalogInputSim sense = new AnalogInputSim(BenchWiring.ANALOG_LAMP_SENSE);
      sense.setVoltage(0.64);
      scenario.run(0.5);
      assertEquals(0.0, bench.lampCommand(), 1e-9, "no lamp command while disabled");
      assertEquals(0.64 * 14.7 / 4.7, bench.lampVolts(), 0.01);
      boolean sawOn = false;
      boolean sawOff = false;
      for (int i = 0; i < 25; i++) {
        scenario.run(0.02);
        sawOn |= bench.heartbeatOn();
        sawOff |= !bench.heartbeatOn();
      }
      assertTrue(sawOn && sawOff, "heartbeat blinks while disabled");

      scenario.enable();
      double max = 0.0;
      double min = 1.0;
      for (int i = 0; i < 250; i++) { // 5 s, more than one sweep
        scenario.run(0.02);
        max = Math.max(max, bench.lampCommand());
        min = Math.min(min, bench.lampCommand());
      }
      assertTrue(max > 0.9, "lamp gets bright: " + max);
      assertTrue(min < 0.1, "lamp gets dim: " + min);
      assertTrue(min >= 0.0 && max <= BenchStand.MAX_LAMP);
    }
  }
}
