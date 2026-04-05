package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

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
    private static final Matrix<N3, N1> odometryStateStdDevs =
            new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Rotation3d> rotationBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(DriveConstants.moduleTranslations);

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
        resetPose(pose, lastWheelPositions);
    }

    public void resetPose(Pose2d pose, SwerveModulePosition[] wheelPositions) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        lastWheelPositions = copyWheelPositions(wheelPositions);
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
        lastWheelPositions = copyWheelPositions(observation.wheelPositions());

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

        double[] r = new double[3];
        for (int i = 0; i < 3; i++) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }

        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());
        Matrix<N3, N1> kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(),
                transform.getY(),
                transform.getRotation().getRadians()));

        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

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

    private SwerveModulePosition[] copyWheelPositions(SwerveModulePosition[] wheelPositions) {
        SwerveModulePosition[] copiedPositions = new SwerveModulePosition[wheelPositions.length];
        for (int i = 0; i < wheelPositions.length; i++) {
            copiedPositions[i] =
                    new SwerveModulePosition(wheelPositions[i].distanceMeters, wheelPositions[i].angle);
        }
        return copiedPositions;
    }
}
