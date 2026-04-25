package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

public class RobotState {
    private static final double poseBufferSizeSec = 2.0;
    private static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
            .createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer = TimeInterpolatableBuffer
            .createBuffer(poseBufferSizeSec);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d gyroOffset = Rotation2d.kZero;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private ChassisSpeeds robotSetpointVelocity = new ChassisSpeeds();

    private RobotState() {
        for (int i = 0; i < 3; i++) {
            qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    public void setRobotVelocity(ChassisSpeeds robotVelocity) {
        this.robotVelocity = robotVelocity;
    }

    public ChassisSpeeds getRobotSetpointVelocity() {
        return robotSetpointVelocity;
    }

    public void setRobotSetpointVelocity(ChassisSpeeds robotSetpointVelocity) {
        this.robotSetpointVelocity = robotSetpointVelocity;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

    public ChassisSpeeds getFieldSetpointVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSetpointVelocity, getRotation());
    }

    public void resetPose(Pose2d pose) {
        resetPose(pose, lastWheelPositions, Optional.empty());
    }

    public void resetPose(Pose2d pose, SwerveModulePosition[] wheelPositions) {
        resetPose(pose, wheelPositions, Optional.empty());
    }

    public void resetPose(Pose2d pose, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> yawMeasurement) {
        if (yawMeasurement.isPresent()) {
            Logger.recordOutput("Auto/firstyaw", yawMeasurement.get().getDegrees());
            Logger.recordOutput("Auto/Robostatefirstpose", pose);
            gyroOffset = pose.getRotation().minus(yawMeasurement.get());
        } else {
            System.out.println("WARNING: Resetting pose without yaw measurement, gyro offset may be inaccurate");
            gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        }
        estimatedPose = pose;
        odometryPose = pose;
        copyWheelPositionsInto(lastWheelPositions, wheelPositions);
        poseBuffer.clear();
        rotationBuffer.clear();
    }

    public void addOdometryObservation(OdometryObservation observation) {
        double tiltScale = 1.0;
        if (observation.pitch().isPresent() && observation.roll().isPresent()) {
            tiltScale = 1.0 - MathUtil.inverseInterpolate(
                    0.0,
                    25.0,
                    Math.abs(Units.radiansToDegrees(
                            Math.acos(observation.pitch().get().getCos() * observation.roll().get().getCos()))));
        }
        tiltScale = MathUtil.clamp(tiltScale, 0.0, 1.0);

        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        twist = new Twist2d(twist.dx * tiltScale, twist.dy * tiltScale, twist.dtheta * tiltScale);
        copyWheelPositionsInto(lastWheelPositions, observation.wheelPositions());

        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);

        observation.yaw().ifPresent(gyroAngle -> {
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

        poseBuffer.addSample(observation.timestamp(), odometryPose);

        if (observation.roll().isPresent() && observation.pitch().isPresent() && observation.yaw().isPresent()) {
            rotationBuffer.addSample(
                    observation.timestamp(),
                    new Rotation3d(
                            observation.roll().get().getRadians(),
                            observation.pitch().get().getRadians(),
                            observation.yaw().get().getRadians()));
        }

        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }

        Optional<Pose2d> sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            return;
        }

        Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        double xVariance = observation.stdDevs().get(0, 0) * observation.stdDevs().get(0, 0);
        double yVariance = observation.stdDevs().get(1, 0) * observation.stdDevs().get(1, 0);
        double thetaVariance = observation.stdDevs().get(2, 0) * observation.stdDevs().get(2, 0);

        double kx = getKalmanGain(qStdDevs.get(0, 0), xVariance);
        double ky = getKalmanGain(qStdDevs.get(1, 0), yVariance);
        double kTheta = getKalmanGain(qStdDevs.get(2, 0), thetaVariance);

        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());
        Transform2d scaledTransform = new Transform2d(
                transform.getX() * kx,
                transform.getY() * ky,
                Rotation2d.fromRadians(transform.getRotation().getRadians() * kTheta));

        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
        Optional<Pose2d> oldOdometryPose = poseBuffer.getSample(timestamp);
        if (oldOdometryPose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get())));
    }

    public Optional<Rotation3d> getEstimatedRotation3dAtTimestamp(double timestamp) {
        return rotationBuffer.getSample(timestamp);
    }

    public record OdometryObservation(
            double timestamp,
            SwerveModulePosition[] wheelPositions,
            Optional<Rotation2d> roll,
            Optional<Rotation2d> pitch,
            Optional<Rotation2d> yaw) {
    }

    public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {
    }

    private static double getKalmanGain(double modelVariance, double measurementVariance) {
        if (modelVariance == 0.0) {
            return 0.0;
        }
        return modelVariance / (modelVariance + Math.sqrt(modelVariance * measurementVariance));
    }

    private static void copyWheelPositionsInto(
            SwerveModulePosition[] destination,
            SwerveModulePosition[] source) {
        for (int i = 0; i < source.length; i++) {
            destination[i].distanceMeters = source[i].distanceMeters;
            destination[i].angle = source[i].angle;
        }
    }
}
