package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class DriveCommand extends CommandBase {
    private static final double SPIN_ROTATIONS_PER_SECOND = 0.2;

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Rotation2d> pigeonYawSupplier;
    private final Supplier<Double> joystickAngleSupplier;
    private final Supplier<Translation2d> joystickMovementSupplier;

    public DriveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Rotation2d> pigeonYawSupplier,
            Supplier<Double> joystickAngleSupplier,
            Supplier<Translation2d> joystickMovementSupplier
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.pigeonYawSupplier = pigeonYawSupplier;
        this.joystickAngleSupplier = joystickAngleSupplier;
        this.joystickMovementSupplier = joystickMovementSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Rotation2d direction = pigeonYawSupplier.get();
        SmartDashboard.putNumber("pyaw", direction.getDegrees());

        Translation2d movement = joystickMovementSupplier.get();

        double leftRightSpin = joystickAngleSupplier.get();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                movement.getX(),
                movement.getY(),
                leftRightSpin * SPIN_ROTATIONS_PER_SECOND,
                direction
        );
        swerveSubsystem.apply(chassisSpeeds);
    }
}
