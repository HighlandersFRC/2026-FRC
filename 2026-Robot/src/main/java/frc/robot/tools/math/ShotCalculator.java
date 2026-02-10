package frc.robot.tools.math;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ShotCalculator {
    public static class ShotSolution {
        public final Rotation2d hoodAngle;
        public final double flywheelRPM;
        public final Rotation2d turretAngle;

        public ShotSolution(Rotation2d hoodAngle, double flywheelRPM, Rotation2d turretAngle) {
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
            this.turretAngle = turretAngle;
        }
    }

    // Input: distance to target in meters, Output: hood angle (Rotation2d)
    private final static InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    // Input: distance to target in meters, Output: flywheel RPM
    private final static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    // Input: distance to target in meters, Output: time of flight in seconds
    private final static InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    // Input: distance to target in meters, Output: hood angle (Rotation2d)
    private final static InterpolatingTreeMap<Double, Rotation2d> feedHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    // Input: distance to target in meters, Output: flywheel RPM
    private final static InterpolatingDoubleTreeMap feedFlywheelMap = new InterpolatingDoubleTreeMap();
    // Input: distance to target in meters, Output: time of flight in seconds
    private final static InterpolatingDoubleTreeMap feedTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        for (double[] shotData : Constants.SetPoints.Shooter.SHOT_MAP) {
            double distanceMeters = shotData[0];
            double hoodAngleDegrees = shotData[1];
            double flywheelRPM = shotData[2];
            double timeOfFlightSeconds = shotData[3];

            // Populate the interpolating maps
            hoodAngleMap.put(distanceMeters, Rotation2d.fromDegrees(hoodAngleDegrees));
            flywheelMap.put(distanceMeters, flywheelRPM);
            timeOfFlightMap.put(distanceMeters, timeOfFlightSeconds);
        }

        for (double[] shotData : Constants.SetPoints.Shooter.FEED_SHOT_MAP) {
            double distanceMeters = shotData[0];
            double hoodAngleDegrees = shotData[1];
            double flywheelRPM = shotData[2];
            double timeOfFlightSeconds = shotData[3];

            // Populate the interpolating maps
            feedHoodAngleMap.put(distanceMeters, Rotation2d.fromDegrees(hoodAngleDegrees));
            feedFlywheelMap.put(distanceMeters, flywheelRPM);
            feedTimeOfFlightMap.put(distanceMeters, timeOfFlightSeconds);
        }
    }

    public static ShotSolution calculateShot(Translation2d turretPosition, Translation2d targetPosition,
            ChassisSpeeds robotVelocity) {
        double distanceToTarget = turretPosition.getDistance(targetPosition);
        double timeOfFlight = timeOfFlightMap.get(distanceToTarget);
        double vx = -robotVelocity.omegaRadiansPerSecond * (Constants.Physical.Shooter.SHOOTER_POSITION.getY());
        double vy = robotVelocity.omegaRadiansPerSecond * (Constants.Physical.Shooter.SHOOTER_POSITION.getX());
        Translation2d tangentialVelocity = new Translation2d(vx, vy);
        Logger.recordOutput("ShotCalculator/TangentialVelocity", tangentialVelocity);
        Translation2d turretVelocity = new Translation2d(
                robotVelocity.vxMetersPerSecond + tangentialVelocity.getX(),
                robotVelocity.vyMetersPerSecond + tangentialVelocity.getY());
        for (int i = 0; i < 20; i++) { // Numerically solve differential equation TODO: find # of iterations that
                                       // converges best
            Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                    turretVelocity.getX(),
                    turretVelocity.getY()).times(-timeOfFlight));
            distanceToTarget = turretPosition.getDistance(predictedTarget);
            timeOfFlight = timeOfFlightMap.get(distanceToTarget);
        }
        Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                turretVelocity.getX(),
                turretVelocity.getY()).times(-timeOfFlight));
        Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(predictedTarget, new Rotation2d()));
        distanceToTarget = turretPosition.getDistance(predictedTarget);
        Rotation2d hoodAngle = hoodAngleMap.get(distanceToTarget);
        double flywheelRPM = flywheelMap.get(distanceToTarget);
        Rotation2d turretAngle = predictedTarget.minus(turretPosition).getAngle();
        return new ShotSolution(
                hoodAngle, flywheelRPM, turretAngle);
    }

    public static ShotSolution calculateFeedShot(Translation2d turretPosition, Translation2d targetPosition,
            ChassisSpeeds robotVelocity) {
        double distanceToTarget = turretPosition.getDistance(targetPosition);
        double timeOfFlight = feedTimeOfFlightMap.get(distanceToTarget);
        double vx = -robotVelocity.omegaRadiansPerSecond * (Constants.Physical.Shooter.SHOOTER_POSITION.getY());
        double vy = robotVelocity.omegaRadiansPerSecond * (Constants.Physical.Shooter.SHOOTER_POSITION.getX());
        Translation2d tangentialVelocity = new Translation2d(vx, vy);
        Logger.recordOutput("ShotCalculator/TangentialVelocity", tangentialVelocity);
        Translation2d turretVelocity = new Translation2d(
                robotVelocity.vxMetersPerSecond + tangentialVelocity.getX(),
                robotVelocity.vyMetersPerSecond + tangentialVelocity.getY());
        for (int i = 0; i < 20; i++) { // Numerically solve differential equation TODO: find # of iterations that
                                       // converges best
            Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                    turretVelocity.getX(),
                    turretVelocity.getY()).times(-timeOfFlight));
            distanceToTarget = turretPosition.getDistance(predictedTarget);
            timeOfFlight = feedTimeOfFlightMap.get(distanceToTarget);
        }
        Translation2d predictedTarget = targetPosition.plus(new Translation2d(
                turretVelocity.getX(),
                turretVelocity.getY()).times(-timeOfFlight));
        Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(predictedTarget, new Rotation2d()));
        distanceToTarget = turretPosition.getDistance(predictedTarget);
        Rotation2d hoodAngle = feedHoodAngleMap.get(distanceToTarget);
        double flywheelRPM = feedFlywheelMap.get(distanceToTarget);
        Rotation2d turretAngle = predictedTarget.minus(turretPosition).getAngle();
        return new ShotSolution(
                hoodAngle, flywheelRPM, turretAngle);
    }
}
