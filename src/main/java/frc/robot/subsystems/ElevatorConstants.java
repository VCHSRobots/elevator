package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The elevator stand's facts, each with where it came from. A value marked UNKNOWN is a stand-in
 * the team has not measured or told us: the sim and the gains are computed from it, so replace it
 * before trusting a run, and above all before any deploy.
 *
 * <p>Sources: the stand's notes in VCHSRobots/EL-Test-Using-Forge-2026 (NOTES.md) and the Elevator
 * Stand's change log: CAN IDs, directions, chain and gear figures, travel.
 */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  // Wiring. CAN IDs and directions as the team stated them for this stand.
  /** Right motor (seen from the back), CAN 10. It leads. */
  public static final int LEADER_CAN_ID = 10;
  /** Left motor, CAN 11. It follows the right. */
  public static final int FOLLOWER_CAN_ID = 11;
  /** The right motor turning counter-clockwise raises the carriage. */
  public static final boolean LEADER_CLOCKWISE_POSITIVE = false;
  /** The left motor turns clockwise to raise it: opposite to the leader (mounted mirrored). */
  public static final boolean FOLLOWER_OPPOSED = true;
  // No limit switches: the stand has hard stops that cannot be hurt however hard the motor drives.

  // Geometry. Two Kraken X44 on a chain, one stage.
  /** 2.875 motor rotations per sprocket-shaft turn (the stand's notes). */
  public static final double GEAR_RATIO = 2.875;
  /**
   * Carriage travel per drum radian, metres: the chain moves 4.0 in per shaft turn, so the
   * effective radius is 4.0 in / 2 pi (0.0162 m). The sprocket's actual diameter is UNKNOWN.
   */
  public static final double DRUM_RADIUS_METERS = 4.0 * 0.0254 / (2 * Math.PI);
  /** UNKNOWN: the carriage's mass. 2 kg is a stand-in, so the sim's gravity load is a guess. */
  public static final double CARRIAGE_MASS_KG = 2.0;
  /** The usable travel: the stick's full position is 8 in above the bottom (the director, 14 Nov). */
  public static final double TRAVEL_METERS = 8.0 * 0.0254;
  /** No margin: the soft limit is the 8 in the director asked for (the hard stop is beyond it). */
  public static final double SOFT_LIMIT_MARGIN_METERS = 0.0;
  /** UNKNOWN: where the carriage's origin is relative to the robot's; a stand-in for the 3D view. */
  public static final Translation3d CARRIAGE_ORIGIN_AT_BOTTOM = new Translation3d(0.0, 0.0, 0.1);

  // Limits on the first runs.
  /**
   * Scales every speed and acceleration and the jog voltage. 0.25 for the first physical runs;
   * raise it only after a person has watched the full travel at this speed.
   */
  public static final double kSlowFirst = 0.25;
  /** 20 A, as the stand's code limits it. */
  public static final double STATOR_CURRENT_LIMIT_AMPS = 20.0;
  /**
   * Chosen to suit the short travel and a stick that moves it: times kSlowFirst this is 6 in/s
   * (0.152 m/s), a little above {@link #STICK_RATE_MPS} so the profile keeps up. The stand's
   * first hardware runs were 1 in/s: this is faster, so a person must watch it first.
   */
  public static final double CRUISE_VELOCITY_MPS = 0.0254 * 24.0;
  public static final double ACCELERATION_MPS2 = 0.0254 * 160.0;
  /** How fast the stick's height may move the target: 0.12 m/s, about 4.7 in/s (not measured). */
  public static final double STICK_RATE_MPS = 0.12;
  public static final double JOG_VOLTS = 3.0;
  /** 0.1 in, as the stand's code treats "arrived". */
  public static final double TOLERANCE_METERS = 0.1 * 0.0254;

  // Gains. Feedforward computed from the motor model as a starting point; tune with SysId.
  public static final DCMotor MOTORS = DCMotor.getKrakenX44(2);
  /** Rotor radians per metre of carriage travel. */
  private static final double ROTOR_RAD_PER_METER = GEAR_RATIO / DRUM_RADIUS_METERS;
  /** Volts to hold the carriage still against gravity. */
  public static final double kG =
      MOTORS.getVoltage(CARRIAGE_MASS_KG * 9.81 * DRUM_RADIUS_METERS / GEAR_RATIO, 0.0);
  /** Volts per metre per second. */
  public static final double kV = ROTOR_RAD_PER_METER / MOTORS.KvRadPerSecPerVolt;
  /** Volts per metre per second squared. */
  public static final double kA =
      MOTORS.getVoltage(CARRIAGE_MASS_KG * DRUM_RADIUS_METERS / GEAR_RATIO, 0.0);
  public static final double kS = 0.05;
  /** Volts per metre of error. */
  public static final double kP = 60.0;
  public static final double kD = 0.0;
}
