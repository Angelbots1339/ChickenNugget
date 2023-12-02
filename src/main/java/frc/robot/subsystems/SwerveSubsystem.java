package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveConfiguration;
import frc.robot.subsystems.drive.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics m_kinematics;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConfiguration.FRONT_LEFT);
        frontRight = new SwerveModule(SwerveConfiguration.FRONT_RIGHT);
        backLeft = new SwerveModule(SwerveConfiguration.BACK_LEFT);
        backRight = new SwerveModule(SwerveConfiguration.BACK_RIGHT);

        m_kinematics = new SwerveDriveKinematics(
                frontLeft.config.position,
                frontRight.config.position,
                backLeft.config.position,
                backRight.config.position
        );
    }

    public void init() {
        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void apply(/* TODO Chassis speed object */) {
        frontLeft.apply(null);
        frontRight.apply(null);
        backLeft.apply(null);
        backRight.apply(null);
    }
}
