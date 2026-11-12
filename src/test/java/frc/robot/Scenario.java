package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.SimRegistry;

/**
 * A headless scripted scenario: the simulated Driver Station enables, commands run for a set
 * time in 20 ms steps on simulated time, and the test asserts on the subsystems' state. The
 * enable here is the simulator's; nothing touches a real robot. Each step also takes 20 ms of
 * wall time, because Phoenix 6's simulated devices run their control loops on the wall clock.
 */
public final class Scenario implements AutoCloseable {
  public static final double STEP_SECONDS = 0.02;

  private Scenario() {}

  public static Scenario start() {
    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("the simulation HAL did not start");
    }
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimRegistry.clear();
    return new Scenario();
  }

  public Scenario enable() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    pause(0.1); // Phoenix devices see the enable a little later
    return this;
  }

  public Scenario disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    return this;
  }

  /** Runs the scheduler (and every subsystem's periodic and simulation step) for this long. */
  public Scenario run(double seconds) {
    return run(seconds, () -> {});
  }

  /** The same, calling {@code afterEachStep} after every step, for assertions along the way. */
  public Scenario run(double seconds, Runnable afterEachStep) {
    long steps = Math.round(seconds / STEP_SECONDS);
    long next = System.nanoTime();
    for (long i = 0; i < steps; i++) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      SimRegistry.update(STEP_SECONDS);
      SimHooks.stepTiming(STEP_SECONDS);
      afterEachStep.run();
      next += (long) (STEP_SECONDS * 1e9);
      pause((next - System.nanoTime()) / 1e9);
    }
    return this;
  }

  private static void pause(double seconds) {
    if (seconds > 0) {
      try {
        Thread.sleep((long) (seconds * 1000));
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }
  }

  @Override
  public void close() {
    disable();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimRegistry.clear();
    SimHooks.resumeTiming();
  }
}
