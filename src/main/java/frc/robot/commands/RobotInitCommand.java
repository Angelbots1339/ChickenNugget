package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This command is designed to initialize the robot on power on. It, nor anything it calls should move any motors or do
 * anything that could cause the robot to move. We return true from {@link Command#runsWhenDisabled()} so we can
 * schedule this command inside the robot container and everything should be ready to go by the time we enable the
 * robot.
 */
public class RobotInitCommand extends CommandBase {
    private boolean isFinished = false;
    private final SwerveSubsystem swerveSubsystem;

    public RobotInitCommand(
            SwerveSubsystem swerveSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(this.swerveSubsystem);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {
        swerveSubsystem.init();
        isFinished = true;
    }

    /**
     * @return true to enable running while disabled
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
