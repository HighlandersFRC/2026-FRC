package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public final class DriveConstants {
    public static final double odometryFrequency = 250.0;

    public static final double moduleX = (Constants.Physical.ROBOT_LENGTH / 2.0) - Constants.Physical.MODULE_OFFSET;
    public static final double moduleY = (Constants.Physical.ROBOT_WIDTH / 2.0) - Constants.Physical.MODULE_OFFSET;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(moduleX, moduleY),
            new Translation2d(moduleX, -moduleY),
            new Translation2d(-moduleX, moduleY),
            new Translation2d(-moduleX, -moduleY)
    };

    private DriveConstants() {
    }
}
