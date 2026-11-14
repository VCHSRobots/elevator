package frc.robot.subsystems;

import static frc.robot.subsystems.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.forge.ForgeAbort;
import frc.robot.sim.SimRegistry;
import java.util.function.DoubleSupplier;

/**
 * A two-Kraken elevator on Phoenix 6: Motion Magic position control in metres, soft limits from
 * the travel, a stator current limit, and every speed scaled by {@code kSlowFirst}. There are no
 * limit switches: the stand has hard stops that cannot hurt it, and the carriage is assumed to
 * rest on the bottom stop whenever the robot is disabled, so while disabled the height is set to
 * zero there. The soft limits (0 and {@link #MAX_HEIGHT_METERS}) are always on. In simulation an {@link ElevatorSimulation} provides the physics; nothing else
 * differs from the robot. Published for AdvantageScope under {@code /Forge/Elevator}: {@code
 * Mechanism} (a Mechanism2d) and {@code Components} (a Pose3d[] struct, the carriage). While
 * Forge's abort flag is set ({@link ForgeAbort}) it commands zero output and holds, enabled or
 * not, and once the flag clears it holds where it is.
 */
@Logged
public class Elevator extends SubsystemBase implements AutoCloseable {
  public static final String TABLE = "/Forge/Elevator";
  /** The highest the code will drive the carriage: the soft limit. */
  public static final double MAX_HEIGHT_METERS = TRAVEL_METERS - SOFT_LIMIT_MARGIN_METERS;

  /** Re-zero only when the height is off by more than this, to spare the CAN bus. */
  private static final double ZERO_DEADBAND_METERS = 0.0005;

  enum Mode {
    HOLD,
    JOG,
  }

  @NotLogged private final TalonFX leader = new TalonFX(LEADER_CAN_ID);
  @NotLogged private final TalonFX follower = new TalonFX(FOLLOWER_CAN_ID);
  @NotLogged private final StatusSignal<Angle> position = leader.getPosition();
  @NotLogged private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
  @NotLogged private final StatusSignal<Voltage> motorVoltage = leader.getMotorVoltage();
  @NotLogged private final StatusSignal<Current> statorCurrent = leader.getStatorCurrent();
  @NotLogged private final MotionMagicVoltage toHeight = new MotionMagicVoltage(0).withEnableFOC(false);
  @NotLogged private final VoltageOut volts = new VoltageOut(0).withEnableFOC(false);
  @NotLogged private final NeutralOut neutral = new NeutralOut();
  @NotLogged private final ElevatorSimulation sim;
  @NotLogged private final ForgeAbort forgeAbort = new ForgeAbort();

  @NotLogged private final Mechanism2d mechanism = new Mechanism2d(0.6, TRAVEL_METERS + 0.4);
  @NotLogged private final MechanismLigament2d carriage;
  @NotLogged private final SendableBuilderImpl mechanismBuilder = new SendableBuilderImpl();
  @NotLogged private final StructArrayPublisher<Pose3d> components;
  /** Where the whole stand sits: always the origin, so the Robot row never moves the frame. */
  @NotLogged private final StructPublisher<Pose3d> robotPose;

  Mode mode = Mode.HOLD;
  double heightMeters;
  double velocityMps;
  double targetMeters;
  double wantedMeters;
  double stickAxis;
  String runningCommand = "";
  double jogVolts;
  double appliedVolts;
  double statorAmps;
  boolean zeroed;
  boolean aborted;

  public Elevator() {
    this(0.0);
  }

  /** @param simStartMeters in simulation only, where the carriage starts; ignored on a robot */
  public Elevator(double simStartMeters) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        LEADER_CLOCKWISE_POSITIVE ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO / (2 * Math.PI * DRUM_RADIUS_METERS);
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_MPS * kSlowFirst;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION_MPS2 * kSlowFirst;
    config.SoftwareLimitSwitch = softLimits();
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(
        new Follower(
            LEADER_CAN_ID, FOLLOWER_OPPOSED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    var root = mechanism.getRoot("base", 0.3, 0.1);
    root.append(new MechanismLigament2d("travel", TRAVEL_METERS, 90, 4, new Color8Bit(80, 80, 80)));
    carriage =
        mechanism.getRoot("carriage", 0.3, 0.1)
            .append(new MechanismLigament2d("height", 0.0, 90, 10, new Color8Bit(255, 160, 0)));
    var nt = NetworkTableInstance.getDefault();
    mechanismBuilder.setTable(nt.getTable(TABLE + "/Mechanism"));
    SendableRegistry.publish(mechanism, mechanismBuilder);
    mechanismBuilder.startListeners();
    components = nt.getStructArrayTopic(TABLE + "/Components", Pose3d.struct).publish();
    robotPose = nt.getStructTopic(TABLE + "/RobotPose", Pose3d.struct).publish();

    if (RobotBase.isSimulation()) {
      sim = new ElevatorSimulation(leader, follower, simStartMeters);
      SimRegistry.register(sim);
    } else {
      sim = null;
    }
  }

  /** Goes to a height in metres, clamped to the travel, and ends there; refused until zeroed. */
  public Command goTo(double meters) {
    return runOnce(
            () -> {
              if (!zeroed) {
                DriverStation.reportWarning("Elevator: home it before going to a height", false);
                return;
              }
              mode = Mode.HOLD;
              targetMeters = MathUtil.clamp(meters, 0.0, MAX_HEIGHT_METERS);
            })
        .andThen(run(() -> {}).until(() -> !zeroed || atTarget()))
        .withName("ElevatorGoTo");
  }

  /** Moves at a fraction (-1 to 1, up positive) of the jog voltage while held, then holds. */
  public Command jog(DoubleSupplier fraction) {
    return runEnd(
            () -> {
              mode = Mode.JOG;
              jogVolts = MathUtil.clamp(fraction.getAsDouble(), -1.0, 1.0) * JOG_VOLTS * kSlowFirst;
            },
            this::holdHere)
        .withName("ElevatorJog");
  }

  /**
   * Follows a stick: the axis (0 to 1, 0 the bottom, 1 the top of the soft limit) is the height
   * wanted, and the target moves toward it at no more than {@code STICK_RATE_MPS}. Starts from
   * where the carriage is, so taking over never jumps. Does nothing until homed. Runs until
   * another command takes the subsystem.
   */
  public Command followAxis(DoubleSupplier axis) {
    var rate = new SlewRateLimiter(STICK_RATE_MPS);
    return runEnd(
            () -> {
              if (!zeroed) {
                return;
              }
              mode = Mode.HOLD;
              stickAxis = axis.getAsDouble();
              double wanted = MathUtil.clamp(stickAxis, 0.0, 1.0) * MAX_HEIGHT_METERS;
              wantedMeters = wanted;
              targetMeters = rate.calculate(wanted);
            },
            this::holdHere)
        .beforeStarting(() -> rate.reset(heightMeters))
        .withName("ElevatorFollowAxis");
  }

  public boolean atTarget() {
    return Math.abs(targetMeters - heightMeters) < TOLERANCE_METERS;
  }

  public double heightMeters() {
    return heightMeters;
  }

  /** In simulation, where the carriage really is (metres); NaN on a robot. */
  public double simHeightMeters() {
    return sim != null ? sim.heightMeters() : Double.NaN;
  }

  public boolean zeroed() {
    return zeroed;
  }

  /** Whether Forge's abort flag held it this cycle. */
  public boolean aborted() {
    return aborted;
  }

  /** The output the leader was last asked for, in volts (0 while neutral). */
  public double appliedVolts() {
    return appliedVolts;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position, velocity, motorVoltage, statorCurrent);
    heightMeters = position.getValueAsDouble();
    velocityMps = velocity.getValueAsDouble();
    appliedVolts = motorVoltage.getValueAsDouble();
    statorAmps = statorCurrent.getValueAsDouble();
    if (DriverStation.isDisabled() && !forgeAbort.isSet()) {
      zeroHere(); // the carriage is assumed to be on the bottom stop at every enable
    }
    aborted = forgeAbort.isSet();
    var running = getCurrentCommand();
    runningCommand = running == null ? "none" : running.getName();

    if (DriverStation.isDisabled() || aborted) {
      // On enable, or once Forge's abort flag clears, hold where the carriage is, not where it
      // was last sent. While the flag is set: zero output, the motors in brake.
      holdHere();
      leader.setControl(neutral);
    } else if (mode == Mode.JOG) {
      // The soft limits stop it at 0 and the top; the hard stops cannot hurt it.
      leader.setControl(volts.withOutput(kG + jogVolts));
    } else {
      drive();
    }
    publish();
  }

  /** Read-backs on the diagnostics table: what the elevator decided this cycle. */
  public void publishDiagnostics(edu.wpi.first.networktables.NetworkTable t) {
    t.getEntry("Elev/Zeroed").setBoolean(zeroed);
    t.getEntry("Elev/Aborted").setBoolean(aborted);
    t.getEntry("Elev/Mode").setString(mode.name());
    t.getEntry("Elev/Running").setString(runningCommand);
    t.getEntry("Elev/StickAxis").setDouble(stickAxis);
    t.getEntry("Elev/WantedMeters").setDouble(wantedMeters);
    t.getEntry("Elev/TargetMeters").setDouble(targetMeters);
    t.getEntry("Elev/HeightMeters").setDouble(heightMeters);
    t.getEntry("Elev/SimHeightMeters").setDouble(simHeightMeters());
    t.getEntry("Elev/AppliedVolts").setDouble(appliedVolts);
    t.getEntry("Elev/StatorAmps").setDouble(statorAmps);
  }

  /** Holds the target; at the bottom it rests on the stop. */
  private void drive() {
    if (targetMeters <= TOLERANCE_METERS && heightMeters <= 2 * TOLERANCE_METERS) {
      leader.setControl(neutral);
      return;
    }
    leader.setControl(toHeight.withPosition(targetMeters));
  }

  private void holdHere() {
    mode = Mode.HOLD;
    targetMeters = heightMeters;
  }

  /** Calls the present position zero (only while disabled), and marks the height known. */
  private void zeroHere() {
    if (Math.abs(heightMeters) > ZERO_DEADBAND_METERS) {
      leader.setPosition(0.0);
      heightMeters = 0.0;
    }
    targetMeters = 0.0;
    zeroed = true;
  }

  private static SoftwareLimitSwitchConfigs softLimits() {
    var limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = MAX_HEIGHT_METERS;
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = 0.0;
    return limits;
  }

  private void publish() {
    carriage.setLength(Math.max(heightMeters, 0.0) + 0.05);
    mechanismBuilder.update();
    robotPose.set(Pose3d.kZero);
    components.set(
        new Pose3d[] {
          new Pose3d(
              CARRIAGE_ORIGIN_AT_BOTTOM.plus(new Translation3d(0.0, 0.0, heightMeters)),
              Rotation3d.kZero)
        });
  }

  @Override
  public void close() {
    CommandScheduler.getInstance().unregisterSubsystem(this);
    if (sim != null) {
      SimRegistry.unregister(sim);
    }
    components.close();
    robotPose.close();
    forgeAbort.close();
    mechanismBuilder.close();
    mechanism.close();
    leader.close();
    follower.close();
  }
}
