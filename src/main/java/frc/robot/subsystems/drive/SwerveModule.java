package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public final SwerveConfiguration config;
    private final TalonFX rotateMotor;
    private final TalonFX moveMotor;
    private final Canandcoder rotationEncoder;
    private final PIDController pidController;
    private double desiredAngle = 0;
    private int count = 0;

    public SwerveModule(SwerveConfiguration config) {
        this.config = config;
        this.rotateMotor = new TalonFX(config.rotateCanId);
        this.moveMotor = new TalonFX(config.moveCanId);
        this.rotationEncoder = new Canandcoder(config.canCoderId);
        this.pidController = new PIDController(0.0095, 0, 0);

        pidController.setTolerance(0.2, 30);
        pidController.enableContinuousInput(0, 360);
    }

    /**
     * Init this module, clearing any sticky faults or performing any other setup that needs to be done before the
     * module is ready to move. This should NEVER actually send power to a motor.
     */
    public void init() {
        this.rotationEncoder.clearStickyFaults();

        SmartDashboard.putNumber("swt", desiredAngle);

        pidController.reset();
    }

    public void apply(SwerveModuleState state) {
        count++;
        if (count >= 500) { // 10 seconds
            count = 0;
            this.pidController.reset();
            desiredAngle = (desiredAngle + 90) % 360;

            SmartDashboard.putNumber("swt", desiredAngle);
        }

        double actualAngle = this.rotationEncoder.getAbsPosition() * 360;
        SmartDashboard.putNumber(config.name + " encoder", actualAngle);

        double rotationPower = pidController.calculate(actualAngle, desiredAngle);
        if(!pidController.atSetpoint()) {
            this.rotateMotor.set(rotationPower);
        }
    }
}
