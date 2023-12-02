// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RobotInitCommand;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Pigeon2 pigeon2;

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        pigeon2 = new Pigeon2(9);
//        pigeon2.configMountPose(Pigeon2.AxisDirection.PositiveY, Pigeon2.AxisDirection.PositiveZ);
        pigeon2.setYaw(0);

        Supplier<Double> joystickRobotSpin = () -> {
            double rightX = m_driverController.getRightX();
            SmartDashboard.putNumber("right X", rightX);

            return rightX;
        };
        Supplier<Rotation2d> robotHeadingAngle = () -> {
            double yaw = pigeon2.getYaw().getValue();
            while (yaw >= 360.0) {
                yaw -= 360.0;
            }
            while (yaw <= 0) {
                yaw += 360.0;
            }
            return Rotation2d.fromDegrees(yaw);
        };
        Supplier<Translation2d> joystickRobotMovement = () -> {
            double x = MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2);
            double y = MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.2);
            SmartDashboard.putNumber("left X", x);
            SmartDashboard.putNumber("left Y", y);
            return new Translation2d(
                    x,
                    y
            );
        };

        // Set the DriveCommand to control the swerve subsystem by default, unless something else needs it.
        CommandScheduler.getInstance().setDefaultCommand(
                swerveSubsystem,
                new DriveCommand(
                        swerveSubsystem,
                        robotHeadingAngle,
                        joystickRobotSpin,
                        joystickRobotMovement
                )
        );

        // Initialize the robot
        (new RobotInitCommand(
                swerveSubsystem
        )).execute();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
//        m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
