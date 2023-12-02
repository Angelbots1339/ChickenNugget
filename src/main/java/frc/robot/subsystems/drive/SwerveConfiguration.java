package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;

public enum SwerveConfiguration {
    FRONT_LEFT(
            "FL",
            new Translation2d(-0.18, 0.18),
            5,
            2,
            6
    ),
    FRONT_RIGHT(
            "FR",
            new Translation2d(0.18, 0.18),
            12,
            20,
            8
    ),
    BACK_LEFT(
            "BL",
            new Translation2d(-0.18, -0.18),
            7,
            18,
            3
    ),
    BACK_RIGHT(
            "BR",
            new Translation2d(0.18, -0.18),
            13,
            19,
            4
    );

    // If gears face inward on wheels at offset, then move motion should be inverted for left from right

    public final String name;
    public final Translation2d position;
    public final int moveCanId;
    public final int rotateCanId;
    public final int canCoderId;

    SwerveConfiguration(
            String name,
            Translation2d position,
            int moveCanId,
            int rotateCanId,
            int CANCoderId
    ) {
        this.name = name;
        this.position = position;
        this.moveCanId = moveCanId;
        this.rotateCanId = rotateCanId;
        this.canCoderId = CANCoderId;
    }
}
