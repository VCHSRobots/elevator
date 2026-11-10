package frc.robot.hardware;

/**
 * One physical unit's hardware map: what differs between two units running the same code.
 * Add the fields this repository's subsystems need (CAN IDs, inversions, encoder offsets).
 */
public record Unit(String name, String serial, boolean known) {}
