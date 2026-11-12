package frc.robot.forge;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Forge's abort flag. When a limit a Forge script declared trips, the station stops Forge's input
 * and sets {@code /Forge/Abort} true (the reason is at {@code /Forge/AbortReason}); it stays set
 * until the next deploy or a person sets it false. Every subsystem reads it each cycle and, while
 * it is set, commands zero output and holds. Forge never disables the robot: a person does.
 */
public final class ForgeAbort implements AutoCloseable {
  public static final String TOPIC = "/Forge/Abort";

  private final BooleanSubscriber flag;

  public ForgeAbort() {
    this(NetworkTableInstance.getDefault());
  }

  public ForgeAbort(NetworkTableInstance nt) {
    flag = nt.getBooleanTopic(TOPIC).subscribe(false);
  }

  /** Whether Forge's abort flag is set now. */
  public boolean isSet() {
    return flag.get();
  }

  @Override
  public void close() {
    flag.close();
  }
}
