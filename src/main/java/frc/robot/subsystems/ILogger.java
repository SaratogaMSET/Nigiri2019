package frc.robot.subsystems;

/**Use this interface for logging:
 * 1. Use the diagnostics method for diagnostic Shuffleboard output to the subsystems tab (I.E. output diganostis for the drivetrain to the "Drive" tab)
 * 2. Use the essentials method for essential Shuffleboard stuff on the main tab (I.E. things that could easily break in the drivetrain to the "Main" tab)
 */
public interface ILogger {
    void diagnosticShuffleboard();
    void essentialShuffleboard();
}