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
import edu.wpi.first.wpilibj.DigitalInput;
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
 * the travel, limit switches on DIO that stop it at either end and zero it at the bottom, a
 * stator current limit, and every speed scaled by {@code kSlowFirst}. Until the bottom switch
 * has been seen the height is unknown: the soft limits stay off and {@link #goTo} refuses; use
 * {@link #home}. In simulation an {@link ElevatorSimulation} provides the physics; nothing else
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

  enum Mode {
    HOLD,
    JOG,
  }

  @NotLogged private final TalonFX leader = new TalonFX(LEADER_CAN_ID);
  @NotLogged private final TalonFX follower = new TalonFX(FOLLOWER_CAN_ID);
  @NotLogged private final DigitalInput bottomSwitch = new DigitalInput(BOTTOM_LIMIT_DIO);
  @NotLogged private final DigitalInput topSwitch = new DigitalInput(TOP_LIMIT_DIO);
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
  double jogVolts;
  double appliedVolts;
  double statorAmps;
  boolean zeroed;
  boolean bottomPressed;
  boolean topPressed;
  boolean aborted;
  private boolean wasBottomPressed;

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
    config.SoftwareLimitSwitch = softLimits(false); // on once the bottom switch has zeroed it
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
      sim = new ElevatorSimulation(leader, follower, bottomSwitch, topSwitch, simStartMeters);
      SimRegistry.register(sim);
    } else {
      sim = null;
    }
  }

  /** Goes to a height in metres, clamped to the travel, and ends there; refused until homed. */
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
              double wanted = MathUtil.clamp(axis.getAsDouble(), 0.0, 1.0) * MAX_HEIGHT_METERS;
              targetMeters = rate.calculate(wanted);
            },
            this::holdHere)
        .beforeStarting(() -> rate.reset(heightMeters))
        .withName("ElevatorFollowAxis");
  }

  /** Lowers slowly onto the bottom switch, which zeroes the height, then holds there. */
  public Command home() {
    return runEnd(
            () -> {
              mode = Mode.JOG;
              jogVolts = -HOME_VOLTS;
            },
            this::holdHere)
        .until(() -> bottomPressed)
        .withTimeout(HOME_TIMEOUT_SECONDS)
        .withName("ElevatorHome");
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

  public boolean bottomPressed() {
    return bottomPressed;
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
    bottomPressed = pressed(bottomSwitch);
    topPressed = pressed(topSwitch);
    if (bottomPressed && !wasBottomPressed) {
      zeroHere();
    }
    wasBottomPressed = bottomPressed;
    aborted = forgeAbort.isSet();

    if (DriverStation.isDisabled() || aborted) {
      // On enable, or once Forge's abort flag clears, hold where the carriage is, not where it
      // was last sent. While the flag is set: zero output, the motors in brake.
      holdHere();
      leader.setControl(neutral);
    } else if (mode == Mode.JOG) {
      if ((jogVolts < 0 && bottomPressed) || (jogVolts > 0 && topPressed)) {
        holdHere();
        drive();
      } else {
        leader.setControl(volts.withOutput(kG + jogVolts));
      }
    } else {
      drive();
    }
    publish();
  }

  /** Holds the target, but rests on the bottom stop and never presses into the top one. */
  private void drive() {
    if (bottomPressed && targetMeters <= heightMeters + TOLERANCE_METERS) {
      leader.setControl(neutral);
      return;
    }
    if (topPressed && targetMeters > heightMeters) {
      targetMeters = heightMeters;
    }
    leader.setControl(toHeight.withPosition(targetMeters));
  }

  private void holdHere() {
    mode = Mode.HOLD;
    targetMeters = heightMeters;
  }

  private void zeroHere() {
    leader.setPosition(0.0);
    targetMeters = Math.max(targetMeters - heightMeters, 0.0); // the same place, in the new frame
    heightMeters = 0.0;
    if (!zeroed) {
      zeroed = true;
      leader.getConfigurator().apply(softLimits(true)); // the follower only copies the leader
    }
  }

  private static SoftwareLimitSwitchConfigs softLimits(boolean enabled) {
    var limits = new SoftwareLimitSwitchConfigs();
    limits.ForwardSoftLimitEnable = enabled;
    limits.ForwardSoftLimitThreshold = MAX_HEIGHT_METERS;
    limits.ReverseSoftLimitEnable = enabled;
    limits.ReverseSoftLimitThreshold = 0.0;
    return limits;
  }

  private static boolean pressed(DigitalInput input) {
    return input.get() != LIMIT_PRESSED_IS_LOW;
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
    bottomSwitch.close();
    topSwitch.close();
    leader.close();
    follower.close();
  }
}
