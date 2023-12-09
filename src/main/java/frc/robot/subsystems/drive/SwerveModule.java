package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public final SwerveConfiguration config;
    public final TalonFX rotateMotor;
    public final TalonFX moveMotor;
    public final CANcoder rotationEncoder;
    public final PIDController pidController;

    public SwerveModule(SwerveConfiguration config) {
        this.config = config;
        this.rotateMotor = new TalonFX(config.rotateCanId);
        this.moveMotor = new TalonFX(config.moveCanId);
        this.rotationEncoder = new CANcoder(config.canCoderId);

        this.pidController = new PIDController(0.0095, 0, 0.00015);
//
        pidController.setTolerance(0.2);
        pidController.enableContinuousInput(0, 360);
    }

    /**
     * Init this module, clearing any sticky faults or performing any other setup that needs to be done before the
     * module is ready to move. This should NEVER actually send power to a motor.
     */
    public void init() {
        this.rotationEncoder.clearStickyFaults();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        this.rotationEncoder.getConfigurator()
                .refresh(magnetSensorConfigs);
        magnetSensorConfigs.MagnetOffset = config.magneticOffset;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        this.rotationEncoder.getConfigurator().apply(magnetSensorConfigs);

        this.rotateMotor.clearStickyFaults();
        this.rotateMotor.setInverted(true);
        this.moveMotor.clearStickyFaults();
        this.moveMotor.setInverted(false);

        pidController.reset();
    }

    public void apply(SwerveModuleState state) {
        double absPosition = this.rotationEncoder.getAbsolutePosition().getValue();
        double actualAngle = absPosition * 360;

        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(actualAngle));

        double desiredAngle = state.angle.getDegrees();

        double movePower = MathUtil.clamp(state.speedMetersPerSecond, -1, 1);
        this.moveMotor.set(movePower);  // TODO Convert this to actual speed


        double rotationPower = pidController.calculate(actualAngle, desiredAngle);

        rotationPower = MathUtil.clamp(rotationPower, -1, 1);

        if (!pidController.atSetpoint()) {
            this.rotateMotor.set(rotationPower);
        }
    }
}
