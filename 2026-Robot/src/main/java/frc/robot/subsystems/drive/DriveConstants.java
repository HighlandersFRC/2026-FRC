package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public final class DriveConstants {
    public static final double odometryFrequency = 250.0;

    public static final double moduleX = (Constants.Physical.ROBOT_LENGTH / 2.0) - Constants.Physical.MODULE_OFFSET;
    public static final double moduleY = (Constants.Physical.ROBOT_WIDTH / 2.0) - Constants.Physical.MODULE_OFFSET;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(moduleX, moduleY).plus(new Translation2d(Constants.Physical.CAD_OFFSET_METERS.getX(),
                    Constants.Physical.CAD_OFFSET_METERS.getY())),
            new Translation2d(moduleX, -moduleY).plus(new Translation2d(Constants.Physical.CAD_OFFSET_METERS.getX(),
                    Constants.Physical.CAD_OFFSET_METERS.getY())),
            new Translation2d(-Constants.Physical.HEX_MODULE_X_OFFSET, Constants.Physical.HEX_MODULE_Y_OFFSET)
                    .plus(new Translation2d(Constants.Physical.CAD_OFFSET_METERS.getX(),
                            Constants.Physical.CAD_OFFSET_METERS.getY())),
            new Translation2d(-Constants.Physical.HEX_MODULE_X_OFFSET, -Constants.Physical.HEX_MODULE_Y_OFFSET)
                    .plus(new Translation2d(Constants.Physical.CAD_OFFSET_METERS.getX(),
                            Constants.Physical.CAD_OFFSET_METERS.getY()))
    };
    public static final double odometryTranslationDeadbandMeters = 5e-5;
    public static final double odometryYawDeadbandRadians = Math.toRadians(0.01);
    public static final double fieldBorderMarginMeters = 0.5;
    public static final double photonSingleTagAmbiguityThreshold = 0.25;
    public static final double photonXyStdDevCoefficient = 0.01;
    public static final double photonThetaStdDevCoefficient = 0.03;
    public static final double maxLimelightTurretMismatchDegrees = 4.0;

    private DriveConstants() {
    }
}
