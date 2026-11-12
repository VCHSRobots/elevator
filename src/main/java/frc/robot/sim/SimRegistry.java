package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;

/**
 * Every mechanism sim in this program. {@code Robot.simulationPeriodic()} (or a test's {@code
 * Scenario}) calls {@link #update} once per loop: each sim steps, then the battery voltage sags
 * by the total current drawn, and the next step's motors see that voltage.
 */
public final class SimRegistry {
  private static final List<MechanismSim> sims = new ArrayList<>();
  private static double batteryVolts = BatterySim.calculateDefaultBatteryLoadedVoltage();

  private SimRegistry() {}

  public static synchronized void register(MechanismSim sim) {
    sims.add(sim);
  }

  public static synchronized void unregister(MechanismSim sim) {
    sims.remove(sim);
  }

  /** Forgets every sim and restores a full battery; a test calls this between scenarios. */
  public static synchronized void clear() {
    sims.clear();
    batteryVolts = BatterySim.calculateDefaultBatteryLoadedVoltage();
    RoboRioSim.setVInVoltage(batteryVolts);
  }

  public static synchronized void update(double dtSeconds) {
    double[] currents = new double[sims.size()];
    for (int i = 0; i < sims.size(); i++) {
      MechanismSim sim = sims.get(i);
      sim.update(dtSeconds);
      currents[i] = sim.currentDrawAmps();
    }
    batteryVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(currents);
    RoboRioSim.setVInVoltage(batteryVolts);
    for (MechanismSim sim : sims) {
      sim.publish();
    }
  }

  /** The simulated battery voltage after the last update. */
  public static synchronized double batteryVolts() {
    return batteryVolts;
  }
}
