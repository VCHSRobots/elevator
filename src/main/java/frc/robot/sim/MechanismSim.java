package frc.robot.sim;

/**
 * The physics behind one subsystem in simulation. A subsystem builds its sim only when
 * {@code RobotBase.isSimulation()} and registers it with {@link SimRegistry}; on a real robot no
 * sim exists, and the subsystem's code is otherwise the same.
 */
public interface MechanismSim {
  /**
   * Steps the physics: read the simulated motor voltages, advance by this many seconds, and
   * write the resulting positions and velocities back to the simulated sensors and motors.
   */
  void update(double dtSeconds);

  /** The current this mechanism draws from the battery now, in amps, for the battery's sag. */
  double currentDrawAmps();

  /** Publishes the simulated truth (not what the sensors report) for AdvantageScope. */
  default void publish() {}
}
