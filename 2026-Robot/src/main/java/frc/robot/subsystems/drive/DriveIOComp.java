package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.tools.logging.BatteryLogger;
import frc.robot.tools.math.Vector;

public class DriveIOComp extends DriveIO {

        private final TalonFX frontRightDriveMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_DRIVE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX frontRightAngleMotor = new TalonFX(Constants.CANInfo.FRONT_RIGHT_ANGLE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX frontLeftDriveMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_DRIVE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX frontLeftAngleMotor = new TalonFX(Constants.CANInfo.FRONT_LEFT_ANGLE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX backLeftDriveMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_DRIVE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX backLeftAngleMotor = new TalonFX(Constants.CANInfo.BACK_LEFT_ANGLE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX backRightDriveMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_DRIVE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final TalonFX backRightAngleMotor = new TalonFX(Constants.CANInfo.BACK_RIGHT_ANGLE_MOTOR_ID,
                        Constants.CANInfo.CANBUS_NAME);

        private final CANcoder frontRightCanCoder = new CANcoder(Constants.CANInfo.FRONT_RIGHT_MODULE_CANCODER_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final CANcoder frontLeftCanCoder = new CANcoder(Constants.CANInfo.FRONT_LEFT_MODULE_CANCODER_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final CANcoder backLeftCanCoder = new CANcoder(Constants.CANInfo.BACK_LEFT_MODULE_CANCODER_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final CANcoder backRightCanCoder = new CANcoder(Constants.CANInfo.BACK_RIGHT_MODULE_CANCODER_ID,
                        Constants.CANInfo.CANBUS_NAME);
        private final Gyro gyro = new Gyro();

        private final BatteryLogger batteryLogger = BatteryLogger.getInstance();

        private final SwerveModule frontRight = new SwerveModule(1, frontRightAngleMotor, frontRightDriveMotor,
                        frontRightCanCoder);
        private final SwerveModule frontLeft = new SwerveModule(2, frontLeftAngleMotor, frontLeftDriveMotor,
                        frontLeftCanCoder);
        private final SwerveModule backLeft = new SwerveModule(3, backLeftAngleMotor, backLeftDriveMotor,
                        backLeftCanCoder);
        private final SwerveModule backRight = new SwerveModule(4, backRightAngleMotor, backRightDriveMotor,
                        backRightCanCoder);

        private final Peripherals peripherals;

        private PhotonPoseEstimator leftFrontPhotonPoseEstimator;
        private PhotonPoseEstimator leftBackPhotonPoseEstimator;
        private PhotonPoseEstimator rightFrontPhotonPoseEstimator;
        private PhotonPoseEstimator rightBackPhotonPoseEstimator;
        private AprilTagFieldLayout aprilTagFieldLayout;
        private int[] allFiducialIds = new int[0];

        private final Transform3d leftFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-9.5614), Constants.inchesToMeters(14.2213),
                                        Constants.inchesToMeters(24.1563)),
                        new Rotation3d(Math.toRadians(-0.5), Math.toRadians(-8.0), Math.toRadians(75.0)));

        private final Transform3d leftBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-11.4675), Constants.inchesToMeters(13.5008),
                                        Constants.inchesToMeters(24.1563)),
                        new Rotation3d(Math.toRadians(0.7), Math.toRadians(-9.0), Math.toRadians(145.0)));

        private final Transform3d rightFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-9.8347), Constants.inchesToMeters(-14.7105),
                                        Constants.inchesToMeters(13.9669)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-26.0), Math.toRadians(275.0)));

        private final Transform3d rightBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-11.4424), Constants.inchesToMeters(11.4633),
                                        Constants.inchesToMeters(24.1563)),
                        new Rotation3d(Math.toRadians(1.7), Math.toRadians(-11.0), Math.toRadians(215.0)));

        private final LinearFilter filterX = LinearFilter.movingAverage(10);
        private final LinearFilter filterY = LinearFilter.movingAverage(10);
        private final LinearFilter filterOmega = LinearFilter.movingAverage(10);
        private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer
                        .createBuffer(2.0);
        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

        private final StatusSignal<Angle> frontLeftDrivePositionSignal;
        private final StatusSignal<Angle> frontRightDrivePositionSignal;
        private final StatusSignal<Angle> backLeftDrivePositionSignal;
        private final StatusSignal<Angle> backRightDrivePositionSignal;
        private final StatusSignal<Angle> frontLeftTurnPositionSignal;
        private final StatusSignal<Angle> frontRightTurnPositionSignal;
        private final StatusSignal<Angle> backLeftTurnPositionSignal;
        private final StatusSignal<Angle> backRightTurnPositionSignal;
        private final StatusSignal<Angle> yawSignal;
        private final StatusSignal<Angle> pitchSignal;
        private final StatusSignal<Angle> rollSignal;

        private final Queue<Double> timestampQueue;
        private final Queue<Double> frontLeftDrivePositionQueue;
        private final Queue<Double> frontRightDrivePositionQueue;
        private final Queue<Double> backLeftDrivePositionQueue;
        private final Queue<Double> backRightDrivePositionQueue;
        private final Queue<Double> frontLeftTurnPositionQueue;
        private final Queue<Double> frontRightTurnPositionQueue;
        private final Queue<Double> backLeftTurnPositionQueue;
        private final Queue<Double> backRightTurnPositionQueue;
        private final Queue<Double> yawPositionQueue;
        private final Queue<Double> pitchPositionQueue;
        private final Queue<Double> rollPositionQueue;

        private ChassisSpeeds wantedChassisSpeeds = new ChassisSpeeds();
        private SwerveModulePosition[] lastAcceptedOdometryPositions = new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
        };
        private Rotation2d lastAcceptedYawPosition = Rotation2d.kZero;
        private boolean hasAcceptedOdometrySample = false;

        private record PendingVisionObservation(
                        String logKey,
                        Pose2d pose,
                        double timestamp,
                        Matrix<N3, N1> stdDevs) {
        }

        private record PredictedFiducial(
                        int id,
                        Pose3d pose,
                        double txDegrees,
                        double tyDegrees,
                        double txNormalized,
                        double tyNormalized,
                        double distanceMeters,
                        double score) {
        }

        private record LimelightTrackingConfig(
                        boolean focused,
                        String reason,
                        int desiredPipeline,
                        float downscale,
                        int priorityTagId,
                        int[] filterIds,
                        double cropXMin,
                        double cropXMax,
                        double cropYMin,
                        double cropYMax,
                        Pose3d[] predictedTagPoses,
                        double[] predictedTagIds,
                        double[] predictedTagTxDegrees,
                        double[] predictedTagTyDegrees,
                        double cropAreaFraction) {
        }

        public DriveIOComp(Peripherals peripherals) {
                this.peripherals = peripherals;

                frontRight.init();
                frontLeft.init();
                backRight.init();
                backLeft.init();
                gyro.init();

                frontLeftDrivePositionSignal = frontLeftDriveMotor.getPosition();
                frontRightDrivePositionSignal = frontRightDriveMotor.getPosition();
                backLeftDrivePositionSignal = backLeftDriveMotor.getPosition();
                backRightDrivePositionSignal = backRightDriveMotor.getPosition();

                frontLeftTurnPositionSignal = frontLeftAngleMotor.getPosition();
                frontRightTurnPositionSignal = frontRightAngleMotor.getPosition();
                backLeftTurnPositionSignal = backLeftAngleMotor.getPosition();
                backRightTurnPositionSignal = backRightAngleMotor.getPosition();

                yawSignal = gyro.getPigeon().getYaw();
                pitchSignal = gyro.getPigeon().getPitch();
                rollSignal = gyro.getPigeon().getRoll();

                BaseStatusSignal.setUpdateFrequencyForAll(
                                DriveConstants.odometryFrequency,
                                frontLeftDrivePositionSignal,
                                frontRightDrivePositionSignal,
                                backLeftDrivePositionSignal,
                                backRightDrivePositionSignal,
                                frontLeftTurnPositionSignal,
                                frontRightTurnPositionSignal,
                                backLeftTurnPositionSignal,
                                backRightTurnPositionSignal,
                                yawSignal,
                                pitchSignal,
                                rollSignal);

                frontLeftDrivePositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(frontLeftDriveMotor.getPosition().clone());
                frontRightDrivePositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(frontRightDriveMotor.getPosition().clone());
                backLeftDrivePositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(backLeftDriveMotor.getPosition().clone());
                backRightDrivePositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(backRightDriveMotor.getPosition().clone());

                frontLeftTurnPositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(frontLeftAngleMotor.getPosition().clone());
                frontRightTurnPositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(frontRightAngleMotor.getPosition().clone());
                backLeftTurnPositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(backLeftAngleMotor.getPosition().clone());
                backRightTurnPositionQueue = PhoenixOdometryThread.getInstance()
                                .registerSignal(backRightAngleMotor.getPosition().clone());

                timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
                yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getPigeon().getYaw());
                pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getPigeon().getPitch());
                rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getPigeon().getRoll());

                try {
                        ParentDevice.optimizeBusUtilizationForAll(
                                        frontRightDriveMotor,
                                        frontRightAngleMotor,
                                        frontRightCanCoder,
                                        frontLeftDriveMotor,
                                        frontLeftAngleMotor,
                                        frontLeftCanCoder,
                                        backLeftDriveMotor,
                                        backLeftAngleMotor,
                                        backLeftCanCoder,
                                        backRightDriveMotor,
                                        backRightAngleMotor,
                                        backRightCanCoder,
                                        gyro.getPigeon());
                } catch (Exception e) {
                        Logger.recordOutput("Drive/OdometryOptimizeError", e.getMessage());
                }

                loadFieldLayout();
                resetPhotonHeadingData(Timer.getFPGATimestamp(), getYaw());
                PhoenixOdometryThread.getInstance().start();
        }

        private void loadFieldLayout() {
                try {
                        aprilTagFieldLayout = new AprilTagFieldLayout(
                                        Filesystem.getDeployDirectory().getPath() + "/" + "2026-rebuilt.json");
                        allFiducialIds = aprilTagFieldLayout.getTags().stream().mapToInt(tag -> tag.ID).toArray();
                        leftFrontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                        leftFrontRobotToCam);
                        leftBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, leftBackRobotToCam);
                        rightFrontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                        rightFrontRobotToCam);
                        rightBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                        rightBackRobotToCam);
                } catch (Exception e) {
                        java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
                }
        }

        @Override
        void zeroIMU() {
                gyro.setYaw(0.0);
                setPosition(new Pose2d(getPosition().getTranslation(), Rotation2d.kZero));
        }

        @Override
        void setYaw(double degrees) {
                gyro.setYaw(degrees);
        }

        @Override
        Rotation2d getYaw() {
                return gyro.getYaw();
        }

        @Override
        void setWheelsStraight() {
                frontRight.setWheelPID(0.0, 0.0);
                frontLeft.setWheelPID(0.0, 0.0);
                backLeft.setWheelPID(0.0, 0.0);
                backRight.setWheelPID(0.0, 0.0);
                wantedChassisSpeeds = new ChassisSpeeds();
        }

        @Override
        protected void setDriveCurrentLimits(double limit) {
                frontRight.setDriveCurrentLimits(limit);
                frontLeft.setDriveCurrentLimits(limit);
                backLeft.setDriveCurrentLimits(limit);
                backRight.setDriveCurrentLimits(limit);
        }

        @Override
        protected void setAngleCurrentLimits(double limit) {
                frontRight.setAngleCurrentLimits(limit);
                frontLeft.setAngleCurrentLimits(limit);
                backLeft.setAngleCurrentLimits(limit);
                backRight.setAngleCurrentLimits(limit);
        }

        @Override
        protected void setPosition(Pose2d pose) {
                Drive.odometryLock.lock();
                try {
                        clearOdometryQueues();
                        BaseStatusSignal.refreshAll(
                                        frontLeftDrivePositionSignal,
                                        frontRightDrivePositionSignal,
                                        backLeftDrivePositionSignal,
                                        backRightDrivePositionSignal,
                                        frontLeftTurnPositionSignal,
                                        frontRightTurnPositionSignal,
                                        backLeftTurnPositionSignal,
                                        backRightTurnPositionSignal);
                        SwerveModulePosition[] currentWheelPositions = getCurrentWheelPositions(
                                        frontLeftDrivePositionSignal.getValueAsDouble(),
                                        frontRightDrivePositionSignal.getValueAsDouble(),
                                        backLeftDrivePositionSignal.getValueAsDouble(),
                                        backRightDrivePositionSignal.getValueAsDouble(),
                                        Rotation2d.fromRotations(frontLeftTurnPositionSignal.getValueAsDouble()),
                                        Rotation2d.fromRotations(frontRightTurnPositionSignal.getValueAsDouble()),
                                        Rotation2d.fromRotations(backLeftTurnPositionSignal.getValueAsDouble()),
                                        Rotation2d.fromRotations(backRightTurnPositionSignal.getValueAsDouble()));
                        setYaw(pose.getRotation().getDegrees());
                        RobotState.getInstance().resetPose(pose, currentWheelPositions);
                        seedAcceptedOdometryState(currentWheelPositions, pose.getRotation());
                        resetPhotonHeadingData(Timer.getFPGATimestamp(), pose.getRotation());
                } finally {
                        Drive.odometryLock.unlock();
                }
        }

        @Override
        protected Pose2d getPosition() {
                return RobotState.getInstance().getEstimatedPose();
        }

        @Override
        protected void drive(Vector velocityVector, double turnVelocity) {
                double yaw = getYaw().getRadians();
                frontLeft.drive(velocityVector, turnVelocity, yaw);
                frontRight.drive(velocityVector, turnVelocity, yaw);
                backLeft.drive(velocityVector, turnVelocity, yaw);
                backRight.drive(velocityVector, turnVelocity, yaw);
                velocityVector = velocityVector.rotate(yaw);
                wantedChassisSpeeds = new ChassisSpeeds(
                                velocityVector.getI(),
                                velocityVector.getJ(),
                                turnVelocity);
        }

        @Override
        protected void driveRobotCentric(Vector velocityVector, double turnRadiansPerSec) {
                frontLeft.drive(velocityVector, turnRadiansPerSec, 0.0);
                frontRight.drive(velocityVector, turnRadiansPerSec, 0.0);
                backLeft.drive(velocityVector, turnRadiansPerSec, 0.0);
                backRight.drive(velocityVector, turnRadiansPerSec, 0.0);
                wantedChassisSpeeds = new ChassisSpeeds(
                                velocityVector.getI(),
                                velocityVector.getJ(),
                                turnRadiansPerSec);
        }

        @Override
        protected void driveCamCentric(Vector velocityVector, double turnRadiansPerSec, double camAngle) {
                frontLeft.drive(velocityVector, turnRadiansPerSec, camAngle);
                frontRight.drive(velocityVector, turnRadiansPerSec, camAngle);
                backLeft.drive(velocityVector, turnRadiansPerSec, camAngle);
                backRight.drive(velocityVector, turnRadiansPerSec, camAngle);
                velocityVector = velocityVector.rotate(camAngle);
                wantedChassisSpeeds = new ChassisSpeeds(
                                velocityVector.getI(),
                                velocityVector.getJ(),
                                turnRadiansPerSec);
        }

        @Override
        protected ChassisSpeeds getChassisSpeeds() {
                return new ChassisSpeeds(filterX.lastValue(), filterY.lastValue(), filterOmega.lastValue());
        }

        @Override
        void update(DriveState currentState) {
                addTurretObservation(Timer.getFPGATimestamp(), Globals.turretAngle);
                updateOdometry();
                updateMeasuredChassisSpeeds();
                updateVision(currentState);

                batteryLogger.reportCurrentUsage("Drive/FrontRight", frontRight.getDriveMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/FrontRightTurn", frontRight.getAngleMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/FrontLeft", frontLeft.getDriveMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/FrontLeftTurn", frontLeft.getAngleMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/BackRight", backRight.getDriveMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/BackRightTurn", backRight.getAngleMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/BackLeft", backLeft.getDriveMotorSupplyCurrent());
                batteryLogger.reportCurrentUsage("Drive/BackLeftTurn", backLeft.getAngleMotorSupplyCurrent());

                Logger.recordOutput("Swerve/Front Right Drive Current", frontRight.getDriveMotorCurrent());
                Logger.recordOutput("Swerve/Front Left Drive Current", frontLeft.getDriveMotorCurrent());
                Logger.recordOutput("Swerve/Back Right Drive Current", backRight.getDriveMotorCurrent());
                Logger.recordOutput("Swerve/Back Left Drive Current", backLeft.getDriveMotorCurrent());
                Logger.recordOutput("Swerve/Front Right Angle Current", frontRight.getAngleMotorCurrent());
                Logger.recordOutput("Swerve/Front Left Angle Current", frontLeft.getAngleMotorCurrent());
                Logger.recordOutput("Swerve/Back Right Angle Current", backRight.getAngleMotorCurrent());
                Logger.recordOutput("Swerve/Back Left Angle Current", backLeft.getAngleMotorCurrent());
                Logger.recordOutput("Robot/pitch", gyro.getPitchDegrees());
                Logger.recordOutput("Robot/roll", gyro.getRollDegrees());
                Logger.recordOutput("Online/Front Right Drive Online", frontRightDriveMotor.isConnected());
                Logger.recordOutput("Online/Front Left Drive Online", frontLeftDriveMotor.isConnected());
                Logger.recordOutput("Online/Back Right Drive Online", backRightDriveMotor.isConnected());
                Logger.recordOutput("Online/Back Left Drive Online", backLeftDriveMotor.isConnected());

                Logger.recordOutput("Online/Front Right Angle Online", frontRightAngleMotor.isConnected());
                Logger.recordOutput("Online/Front Left Angle Online", frontLeftAngleMotor.isConnected());
                Logger.recordOutput("Online/Back Right Angle Online", backRightAngleMotor.isConnected());
                Logger.recordOutput("Online/Back Left Angle Online", backLeftAngleMotor.isConnected());

                Logger.recordOutput("Online/Front Right CanCoder Online", frontRightCanCoder.isConnected());
                Logger.recordOutput("Online/Front Left CanCoder Online", frontLeftCanCoder.isConnected());
                Logger.recordOutput("Online/Back Right CanCoder Online", backRightCanCoder.isConnected());
                Logger.recordOutput("Online/Back Left CanCoder Online", backLeftCanCoder.isConnected());

                Logger.recordOutput("Pigeon Online", gyro.isOnline());
        }

        @Override
        protected ChassisSpeeds getWantedChassisSpeeds() {
                return wantedChassisSpeeds;
        }

        @Override
        protected boolean getFlat() {
                return Math.abs(gyro.getPitchDegrees()) < 3.5 && Math.abs(gyro.getRollDegrees()) < 3.5;
        }

        private void clearOdometryQueues() {
                timestampQueue.clear();
                frontLeftDrivePositionQueue.clear();
                frontRightDrivePositionQueue.clear();
                backLeftDrivePositionQueue.clear();
                backRightDrivePositionQueue.clear();
                frontLeftTurnPositionQueue.clear();
                frontRightTurnPositionQueue.clear();
                backLeftTurnPositionQueue.clear();
                backRightTurnPositionQueue.clear();
                yawPositionQueue.clear();
                pitchPositionQueue.clear();
                rollPositionQueue.clear();
        }

        private void updateOdometry() {
                double[] timestamps;
                double[] frontLeftDrivePositions;
                double[] frontRightDrivePositions;
                double[] backLeftDrivePositions;
                double[] backRightDrivePositions;
                Rotation2d[] frontLeftTurnPositions;
                Rotation2d[] frontRightTurnPositions;
                Rotation2d[] backLeftTurnPositions;
                Rotation2d[] backRightTurnPositions;
                Rotation2d[] yawPositions;
                Rotation2d[] pitchPositions;
                Rotation2d[] rollPositions;

                Drive.odometryLock.lock();
                try {
                        timestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
                        frontLeftDrivePositions = frontLeftDrivePositionQueue.stream().mapToDouble(Double::doubleValue)
                                        .toArray();
                        frontRightDrivePositions = frontRightDrivePositionQueue.stream()
                                        .mapToDouble(Double::doubleValue).toArray();
                        backLeftDrivePositions = backLeftDrivePositionQueue.stream().mapToDouble(Double::doubleValue)
                                        .toArray();
                        backRightDrivePositions = backRightDrivePositionQueue.stream().mapToDouble(Double::doubleValue)
                                        .toArray();

                        frontLeftTurnPositions = frontLeftTurnPositionQueue.stream().map(Rotation2d::fromRotations)
                                        .toArray(Rotation2d[]::new);
                        frontRightTurnPositions = frontRightTurnPositionQueue.stream().map(Rotation2d::fromRotations)
                                        .toArray(Rotation2d[]::new);
                        backLeftTurnPositions = backLeftTurnPositionQueue.stream().map(Rotation2d::fromRotations)
                                        .toArray(Rotation2d[]::new);
                        backRightTurnPositions = backRightTurnPositionQueue.stream().map(Rotation2d::fromRotations)
                                        .toArray(Rotation2d[]::new);

                        yawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees)
                                        .toArray(Rotation2d[]::new);
                        pitchPositions = pitchPositionQueue.stream().map(Rotation2d::fromDegrees)
                                        .toArray(Rotation2d[]::new);
                        rollPositions = rollPositionQueue.stream().map(Rotation2d::fromDegrees)
                                        .toArray(Rotation2d[]::new);

                        clearOdometryQueues();
                } finally {
                        Drive.odometryLock.unlock();
                }

                int sampleCount = timestamps.length;
                sampleCount = Math.min(sampleCount, frontLeftDrivePositions.length);
                sampleCount = Math.min(sampleCount, frontRightDrivePositions.length);
                sampleCount = Math.min(sampleCount, backLeftDrivePositions.length);
                sampleCount = Math.min(sampleCount, backRightDrivePositions.length);
                sampleCount = Math.min(sampleCount, frontLeftTurnPositions.length);
                sampleCount = Math.min(sampleCount, frontRightTurnPositions.length);
                sampleCount = Math.min(sampleCount, backLeftTurnPositions.length);
                sampleCount = Math.min(sampleCount, backRightTurnPositions.length);
                sampleCount = Math.min(sampleCount, yawPositions.length);
                sampleCount = Math.min(sampleCount, pitchPositions.length);
                sampleCount = Math.min(sampleCount, rollPositions.length);
                int acceptedSamples = 0;

                for (int i = 0; i < sampleCount; i++) {
                        SwerveModulePosition[] wheelPositions = getCurrentWheelPositions(
                                        frontLeftDrivePositions[i],
                                        frontRightDrivePositions[i],
                                        backLeftDrivePositions[i],
                                        backRightDrivePositions[i],
                                        frontLeftTurnPositions[i],
                                        frontRightTurnPositions[i],
                                        backLeftTurnPositions[i],
                                        backRightTurnPositions[i]);
                        Rotation2d yawPosition = yawPositions[i];
                        if (!shouldAcceptOdometrySample(wheelPositions, yawPosition)) {
                                continue;
                        }

                        RobotState.getInstance().addOdometryObservation(new RobotState.OdometryObservation(
                                        timestamps[i],
                                        wheelPositions,
                                        Optional.of(rollPositions[i]),
                                        Optional.of(pitchPositions[i]),
                                        Optional.of(yawPosition)));
                        seedAcceptedOdometryState(wheelPositions, yawPosition);
                        acceptedSamples++;
                }

                Logger.recordOutput("Drive/OdometryQueuedSamples", sampleCount);
                Logger.recordOutput("Drive/OdometryAcceptedSamples", acceptedSamples);
        }

        private void updateMeasuredChassisSpeeds() {
                ChassisSpeeds robotSpeeds = kinematics.toChassisSpeeds(
                                frontLeft.getSwerveModuleState(getYaw()),
                                frontRight.getSwerveModuleState(getYaw()),
                                backLeft.getSwerveModuleState(getYaw()),
                                backRight.getSwerveModuleState(getYaw()));

                filterX.calculate(robotSpeeds.vxMetersPerSecond);
                filterY.calculate(robotSpeeds.vyMetersPerSecond);
                filterOmega.calculate(Math.toRadians(gyro.getAngularVelocityZWorldDegPerSec()));
        }

        private void updateVision(DriveState currentState) {
                updateLimelightTrackingState();

                if (leftFrontPhotonPoseEstimator == null
                                || leftBackPhotonPoseEstimator == null
                                || rightFrontPhotonPoseEstimator == null
                                || rightBackPhotonPoseEstimator == null) {
                        return;
                }

                boolean tilted = Math.abs(gyro.getPitchDegrees()) > 4.1 || Math.abs(gyro.getRollDegrees()) > 4.1;
                if (tilted) {
                        return;
                }

                if (Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) >= 2.4) {
                        return;
                }

                addPhotonHeadingData(Timer.getFPGATimestamp(), RobotState.getInstance().getRotation());

                List<PendingVisionObservation> pendingVisionObservations = new ArrayList<>();
                List<Pose2d> acceptedPhotonPoses = new ArrayList<>();

                processPhotonResults(
                                "Cameras/Right Front Pose",
                                peripherals.getRightFrontCamResults(),
                                rightFrontPhotonPoseEstimator,
                                pendingVisionObservations,
                                acceptedPhotonPoses);

                if (currentState != DriveState.DRIVE_TO_ALIGN_CLIMB && currentState != DriveState.DRIVE_TO_PRE_CLIMB) {
                        processPhotonResults(
                                        "Cameras/Right Back Pose",
                                        peripherals.getRightBackCamResults(),
                                        rightBackPhotonPoseEstimator,
                                        pendingVisionObservations,
                                        acceptedPhotonPoses);

                        processPhotonResults(
                                        "Cameras/Left Back Pose",
                                        peripherals.getLeftBackCamResults(),
                                        leftBackPhotonPoseEstimator,
                                        pendingVisionObservations,
                                        acceptedPhotonPoses);

                        processPhotonResults(
                                        "Cameras/Left Front Pose",
                                        peripherals.getLeftFrontCamResults(),
                                        leftFrontPhotonPoseEstimator,
                                        pendingVisionObservations,
                                        acceptedPhotonPoses);

                        updateLimelightObservation(pendingVisionObservations);
                }

                pendingVisionObservations.stream()
                                .sorted(Comparator.comparingDouble(PendingVisionObservation::timestamp))
                                .forEach(observation -> RobotState.getInstance().addVisionObservation(
                                                new RobotState.VisionObservation(
                                                                observation.timestamp(),
                                                                new Pose3d(observation.pose()),
                                                                observation.stdDevs())));

                Logger.recordOutput("Cameras/Photon Accepted Poses", acceptedPhotonPoses.toArray(Pose2d[]::new));
                Logger.recordOutput("Cameras/Vision Observation Count", pendingVisionObservations.size());
        }

        private void processPhotonResults(
                        String logKey,
                        List<PhotonPipelineResult> results,
                        PhotonPoseEstimator estimator,
                        List<PendingVisionObservation> pendingVisionObservations,
                        List<Pose2d> acceptedPhotonPoses) {
                Pose2d loggedPose = new Pose2d();
                for (PhotonPipelineResult result : results) {
                        Optional<PendingVisionObservation> observation = createPhotonObservation(logKey, estimator,
                                        result);
                        if (observation.isPresent()) {
                                pendingVisionObservations.add(observation.get());
                                acceptedPhotonPoses.add(observation.get().pose());
                                loggedPose = observation.get().pose();
                        }
                }
                Logger.recordOutput(logKey, loggedPose);
                Logger.recordOutput(logKey + "/UnreadResults", results.size());
        }

        private Optional<PendingVisionObservation> createPhotonObservation(
                        String logKey,
                        PhotonPoseEstimator estimator,
                        PhotonPipelineResult result) {
                if (!result.hasTargets() || result.getTimestampSeconds() <= 0.0) {
                        return Optional.empty();
                }

                Optional<EstimatedRobotPose> estimate = estimatePhotonPose(estimator, result);
                if (estimate.isEmpty()) {
                        return Optional.empty();
                }

                boolean ambiguousSingleTag = result.getTargets().size() == 1
                                && result.getBestTarget()
                                                .getPoseAmbiguity() >= DriveConstants.photonSingleTagAmbiguityThreshold;
                if (ambiguousSingleTag) {
                        return Optional.empty();
                }

                Pose2d robotPose = estimate.get().estimatedPose.toPose2d();
                if (!poseInField(robotPose)) {
                        return Optional.empty();
                }

                Logger.recordOutput(logKey + "/LatencySecs", Timer.getFPGATimestamp() - result.getTimestampSeconds());
                return Optional.of(
                                new PendingVisionObservation(
                                                logKey,
                                                robotPose,
                                                result.getTimestampSeconds(),
                                                createPhotonVisionStdDevs(
                                                                getAverageTagDistanceMeters(result),
                                                                result.getTargets().size(),
                                                                estimate.get().strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)));
        }

        private Optional<EstimatedRobotPose> estimatePhotonPose(PhotonPoseEstimator estimator,
                        PhotonPipelineResult result) {
                Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
                if (estimate.isPresent()) {
                        return estimate;
                }

                estimate = estimator.estimatePnpDistanceTrigSolvePose(result);
                if (estimate.isPresent()) {
                        return estimate;
                }

                Optional<Pose2d> referencePose = RobotState.getInstance()
                                .getEstimatedPoseAtTimestamp(result.getTimestampSeconds());
                if (referencePose.isPresent()) {
                        estimate = estimator.estimateClosestToReferencePose(result, new Pose3d(referencePose.get()));
                        if (estimate.isPresent()) {
                                return estimate;
                        }
                }

                return estimator.estimateLowestAmbiguityPose(result);
        }

        private void updateLimelightTrackingState() {
                try {
                        Rotation2d currentTurretAngle = Globals.turretAngle;
                        Pose3d limelightRobotPose = Constants.Vision.updateLimelightPoseFromTurret(
                                        new Pose3d(Constants.Physical.Shooter.SHOOTER_POSITION, Rotation3d.kZero),
                                        currentTurretAngle,
                                        Constants.Vision.turretToLimelight,
                                        Constants.Vision.LIMELIGHT_NAME);
                        Pose3d limelightFieldPose = new Pose3d(getPosition())
                                        .transformBy(new Transform3d(
                                                        limelightRobotPose.getTranslation(),
                                                        limelightRobotPose.getRotation()));

                        double limelightAngVelRelToField = Constants.Vision.getLimelightAngVelRelToField(
                                        Globals.turretVelocity,
                                        getChassisSpeeds().omegaRadiansPerSecond);
                        Logger.recordOutput("Limelight Ang Vel", limelightAngVelRelToField);
                        Logger.recordOutput("Vision/Limelight/AngularVelocity", limelightAngVelRelToField);

                        LimelightTrackingConfig config = buildLimelightTrackingConfig(limelightFieldPose,
                                        limelightAngVelRelToField);

                        LimelightHelpers.SetRobotOrientation_NoFlush(
                                        Constants.Vision.LIMELIGHT_NAME,
                                        getPosition().getRotation().getDegrees(),
                                        limelightAngVelRelToField,
                                        gyro.getPitchDegrees(),
                                        0.0,
                                        -gyro.getRollDegrees(),
                                        0.0);
                        LimelightHelpers.setPipelineIndex(Constants.Vision.LIMELIGHT_NAME, config.desiredPipeline());
                        LimelightHelpers.setPriorityTagID(Constants.Vision.LIMELIGHT_NAME, config.priorityTagId());
                        LimelightHelpers.SetFiducialIDFiltersOverride(
                                        Constants.Vision.LIMELIGHT_NAME,
                                        config.filterIds().length > 0 ? config.filterIds() : allFiducialIds);
                        LimelightHelpers.setCropWindow(
                                        Constants.Vision.LIMELIGHT_NAME,
                                        config.cropXMin(),
                                        config.cropXMax(),
                                        config.cropYMin(),
                                        config.cropYMax());
                        LimelightHelpers.SetFiducialDownscalingOverride(
                                        Constants.Vision.LIMELIGHT_NAME,
                                        config.downscale());
                        LimelightHelpers.Flush();

                        logLimelightTrackingConfig(limelightRobotPose, limelightFieldPose, config);
                } catch (Exception e) {
                        Logger.recordOutput("Vision/Limelight/SmartCrop/Error", e.getMessage());
                }
        }

        private LimelightTrackingConfig buildLimelightTrackingConfig(Pose3d limelightPose, double limelightAngVel) {
                if (aprilTagFieldLayout == null) {
                        return createSearchLimelightConfig("NoFieldLayout");
                }
                if (!poseInField(limelightPose.toPose2d())) {
                        return createSearchLimelightConfig("CameraPoseOutOfField");
                }
                if (Math.abs(limelightAngVel) >= 0.5) {
                        return createSearchLimelightConfig("AngularVelocityTooHigh");
                }

                List<PredictedFiducial> predictedFiducials = predictVisibleLimelightTags(limelightPose);
                if (predictedFiducials.isEmpty()) {
                        return createSearchLimelightConfig("NoPredictedTags");
                }

                double cropXMin = 1.0;
                double cropXMax = -1.0;
                double cropYMin = 1.0;
                double cropYMax = -1.0;
                Pose3d[] predictedTagPoses = new Pose3d[predictedFiducials.size()];
                double[] predictedTagIds = new double[predictedFiducials.size()];
                double[] predictedTagTxDegrees = new double[predictedFiducials.size()];
                double[] predictedTagTyDegrees = new double[predictedFiducials.size()];
                int[] filterIds = new int[Math.min(predictedFiducials.size(),
                                Constants.Vision.LIMELIGHT_MAX_FILTER_TAGS)];

                for (int i = 0; i < predictedFiducials.size(); i++) {
                        PredictedFiducial predictedFiducial = predictedFiducials.get(i);
                        double tagPadding = MathUtil.clamp(
                                        Constants.Vision.LIMELIGHT_CROP_BASE_PADDING
                                                        + Constants.Vision.LIMELIGHT_CROP_CLOSE_TAG_EXTRA_PADDING
                                                                        / (predictedFiducial.distanceMeters() + 1.0),
                                        Constants.Vision.LIMELIGHT_CROP_BASE_PADDING,
                                        0.55);
                        cropXMin = Math.min(cropXMin, predictedFiducial.txNormalized() - tagPadding);
                        cropXMax = Math.max(cropXMax, predictedFiducial.txNormalized() + tagPadding);
                        cropYMin = Math.min(cropYMin, predictedFiducial.tyNormalized() - tagPadding);
                        cropYMax = Math.max(cropYMax, predictedFiducial.tyNormalized() + tagPadding);

                        predictedTagPoses[i] = predictedFiducial.pose();
                        predictedTagIds[i] = predictedFiducial.id();
                        predictedTagTxDegrees[i] = predictedFiducial.txDegrees();
                        predictedTagTyDegrees[i] = predictedFiducial.tyDegrees();
                        if (i < filterIds.length) {
                                filterIds[i] = predictedFiducial.id();
                        }
                }

                double cropCenterX = (cropXMin + cropXMax) / 2.0;
                double cropCenterY = (cropYMin + cropYMax) / 2.0;
                double cropHalfWidth = Math.max(
                                (cropXMax - cropXMin) / 2.0,
                                Constants.Vision.LIMELIGHT_MIN_CROP_SPAN_X / 2.0);
                double cropHalfHeight = Math.max(
                                (cropYMax - cropYMin) / 2.0,
                                Constants.Vision.LIMELIGHT_MIN_CROP_SPAN_Y / 2.0);

                cropCenterX = MathUtil.clamp(cropCenterX, -1.0 + cropHalfWidth, 1.0 - cropHalfWidth);
                cropCenterY = MathUtil.clamp(cropCenterY, -1.0 + cropHalfHeight, 1.0 - cropHalfHeight);
                cropXMin = MathUtil.clamp(cropCenterX - cropHalfWidth, -1.0, 1.0);
                cropXMax = MathUtil.clamp(cropCenterX + cropHalfWidth, -1.0, 1.0);
                cropYMin = MathUtil.clamp(cropCenterY - cropHalfHeight, -1.0, 1.0);
                cropYMax = MathUtil.clamp(cropCenterY + cropHalfHeight, -1.0, 1.0);

                double cropAreaFraction = ((cropXMax - cropXMin) * (cropYMax - cropYMin)) / 4.0;
                float downscale = cropAreaFraction < 0.16
                                ? Constants.Vision.LIMELIGHT_TIGHT_CROP_DOWNSCALE
                                : Constants.Vision.LIMELIGHT_TRACKING_DOWNSCALE;

                return new LimelightTrackingConfig(
                                true,
                                "Tracking",
                                Constants.Vision.LIMELIGHT_TRACKING_PIPELINE,
                                downscale,
                                predictedFiducials.get(0).id(),
                                filterIds,
                                cropXMin,
                                cropXMax,
                                cropYMin,
                                cropYMax,
                                predictedTagPoses,
                                predictedTagIds,
                                predictedTagTxDegrees,
                                predictedTagTyDegrees,
                                cropAreaFraction);
        }

        private List<PredictedFiducial> predictVisibleLimelightTags(Pose3d limelightPose) {
                List<PredictedFiducial> predictedFiducials = new ArrayList<>();
                if (aprilTagFieldLayout == null) {
                        return predictedFiducials;
                }

                double halfHorizontalFov = Constants.Vision.LIMELIGHT_HORIZONTAL_FOV_DEGREES / 2.0;
                double halfVerticalFov = Constants.Vision.LIMELIGHT_VERTICAL_FOV_DEGREES / 2.0;
                for (var tag : aprilTagFieldLayout.getTags()) {
                        Pose3d tagPose = tag.pose;
                        Pose3d tagInCameraFrame = tagPose.relativeTo(limelightPose);
                        double forwardDistance = tagInCameraFrame.getX();
                        if (forwardDistance <= Constants.Vision.LIMELIGHT_MIN_FORWARD_DISTANCE_METERS) {
                                continue;
                        }

                        double lateralDistance = tagInCameraFrame.getY();
                        double verticalDistance = tagInCameraFrame.getZ();
                        double distanceMeters = tagInCameraFrame.getTranslation().getNorm();
                        if (distanceMeters > Constants.Vision.LIMELIGHT_MAX_TRACKING_DISTANCE_METERS) {
                                continue;
                        }

                        double txDegrees = Math.toDegrees(Math.atan2(lateralDistance, forwardDistance));
                        double tyDegrees = Math.toDegrees(Math.atan2(verticalDistance, forwardDistance));
                        if (Math.abs(txDegrees) > halfHorizontalFov
                                        + Constants.Vision.LIMELIGHT_PREDICTION_MARGIN_DEGREES
                                        || Math.abs(tyDegrees) > halfVerticalFov
                                                        + Constants.Vision.LIMELIGHT_PREDICTION_MARGIN_DEGREES) {
                                continue;
                        }

                        double txNormalized = MathUtil.clamp(txDegrees / halfHorizontalFov, -1.0, 1.0);
                        // Bias the crop slightly downward so the tag sits nearer the center of the
                        // crop window instead of hugging the top edge.
                        double tyNormalized = MathUtil.clamp(
                                        (tyDegrees / halfVerticalFov) + Constants.Vision.LIMELIGHT_CROP_Y_BIAS,
                                        -1.0,
                                        1.0);
                        double score = distanceMeters
                                        + 0.015 * Math.abs(txDegrees)
                                        + 0.01 * Math.abs(tyDegrees);

                        predictedFiducials.add(new PredictedFiducial(
                                        tag.ID,
                                        tagPose,
                                        txDegrees,
                                        tyDegrees,
                                        txNormalized,
                                        tyNormalized,
                                        distanceMeters,
                                        score));
                }

                predictedFiducials.sort(Comparator.comparingDouble(PredictedFiducial::score));
                if (predictedFiducials.size() > Constants.Vision.LIMELIGHT_MAX_FILTER_TAGS) {
                        return new ArrayList<>(
                                        predictedFiducials.subList(0, Constants.Vision.LIMELIGHT_MAX_FILTER_TAGS));
                }
                return predictedFiducials;
        }

        private LimelightTrackingConfig createSearchLimelightConfig(String reason) {
                return new LimelightTrackingConfig(
                                false,
                                reason,
                                Constants.Vision.LIMELIGHT_SEARCH_PIPELINE,
                                Constants.Vision.LIMELIGHT_SEARCH_DOWNSCALE,
                                0,
                                allFiducialIds,
                                -1.0,
                                1.0,
                                -1.0,
                                1.0,
                                new Pose3d[0],
                                new double[0],
                                new double[0],
                                new double[0],
                                1.0);
        }

        private void logLimelightTrackingConfig(Pose3d limelightRobotPose, Pose3d limelightFieldPose,
                        LimelightTrackingConfig config) {
                Logger.recordOutput("Vision/Limelight/CameraPoseRobot", limelightRobotPose);
                Logger.recordOutput("Vision/Limelight/CameraPose", limelightFieldPose);
                Logger.recordOutput("Vision/Limelight/SmartCrop/Focused", config.focused());
                Logger.recordOutput("Vision/Limelight/SmartCrop/Reason", config.reason());
                Logger.recordOutput("Vision/Limelight/SmartCrop/CropWindow",
                                new double[] {
                                                config.cropXMin(),
                                                config.cropXMax(),
                                                config.cropYMin(),
                                                config.cropYMax()
                                });
                Logger.recordOutput("Vision/Limelight/SmartCrop/CropAreaFraction", config.cropAreaFraction());
                Logger.recordOutput("Vision/Limelight/SmartCrop/PriorityTagId", config.priorityTagId());
                Logger.recordOutput("Vision/Limelight/SmartCrop/FilterTagIds",
                                toDoubleArray(config.filterIds()));
                Logger.recordOutput("Vision/Limelight/SmartCrop/PredictedTagIds", config.predictedTagIds());
                Logger.recordOutput("Vision/Limelight/SmartCrop/PredictedTagTxDegrees", config.predictedTagTxDegrees());
                Logger.recordOutput("Vision/Limelight/SmartCrop/PredictedTagTyDegrees", config.predictedTagTyDegrees());
                Logger.recordOutput("Vision/Limelight/SmartCrop/PredictedTagPoses", config.predictedTagPoses());
                Logger.recordOutput("Vision/Limelight/Pipeline/Desired", config.desiredPipeline());
                Logger.recordOutput("Vision/Limelight/Pipeline/Active",
                                LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.LIMELIGHT_NAME));
                Logger.recordOutput("Vision/Limelight/Downscale", config.downscale());

                LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers
                                .getRawFiducials(Constants.Vision.LIMELIGHT_NAME);
                double[] rawIds = new double[rawFiducials.length];
                double[] rawTxDegrees = new double[rawFiducials.length];
                double[] rawTyDegrees = new double[rawFiducials.length];
                for (int i = 0; i < rawFiducials.length; i++) {
                        rawIds[i] = rawFiducials[i].id;
                        rawTxDegrees[i] = rawFiducials[i].txnc;
                        rawTyDegrees[i] = rawFiducials[i].tync;
                }
                Logger.recordOutput("Vision/Limelight/Raw/Ids", rawIds);
                Logger.recordOutput("Vision/Limelight/Raw/TxDegrees", rawTxDegrees);
                Logger.recordOutput("Vision/Limelight/Raw/TyDegrees", rawTyDegrees);
        }

        private double[] toDoubleArray(int[] values) {
                double[] converted = new double[values.length];
                for (int i = 0; i < values.length; i++) {
                        converted[i] = values[i];
                }
                return converted;
        }

        private void updateLimelightObservation(List<PendingVisionObservation> pendingVisionObservations) {
                double limelightAngVelRelToField = Constants.Vision.getLimelightAngVelRelToField(
                                Globals.turretVelocity,
                                getChassisSpeeds().omegaRadiansPerSecond);
                Logger.recordOutput("Limelight Ang Vel", limelightAngVelRelToField);
                if (Math.abs(limelightAngVelRelToField) >= 0.5) {
                        return;
                }

                try {
                        Rotation2d currentTurretAngle = Globals.turretAngle;
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT_NAME);
                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue(Constants.Vision.LIMELIGHT_NAME);
                        Logger.recordOutput("Cameras/Limelight Pose MT1", mt1 != null ? mt1.pose : new Pose2d());

                        if (mt1 == null) {
                                Logger.recordOutput("Cameras/Limelight Pose", new Pose2d());
                                return;
                        }

                        if (mt1.timestampSeconds <= 0.0) {
                                Logger.recordOutput("Cameras/Limelight Pose", new Pose2d());
                                return;
                        }

                        Rotation2d measurementTurretAngle = turretAngleBuffer.getSample(mt1.timestampSeconds)
                                        .orElse(currentTurretAngle);
                        if (Math.abs(measurementTurretAngle.minus(currentTurretAngle)
                                        .getDegrees()) > DriveConstants.maxLimelightTurretMismatchDegrees) {
                                Logger.recordOutput("Cameras/Limelight Pose", new Pose2d());
                                return;
                        }

                        if (mt1.tagCount == 0 || mt1.avgTagDist > 4.5 || !poseInField(mt1.pose)) {
                                Logger.recordOutput("Cameras/Limelight Pose", new Pose2d());
                                return;
                        }

                        Logger.recordOutput("Limelight dist to tag", mt1.avgTagDist);
                        Logger.recordOutput("Cameras/Limelight Pose", mt1.pose);
                        pendingVisionObservations.add(new PendingVisionObservation(
                                        "Cameras/Limelight Pose",
                                        mt1.pose,
                                        mt1.timestampSeconds,
                                        createVisionStdDevs(mt1.avgTagDist, mt1.tagCount, 2.0)));
                } catch (Exception e) {
                        System.out.println(e);
                }
        }

        private void addTurretObservation(double timestamp, Rotation2d turretAngle) {
                turretAngleBuffer.addSample(timestamp, turretAngle);
        }

        private boolean shouldAcceptOdometrySample(SwerveModulePosition[] wheelPositions, Rotation2d yawPosition) {
                if (!hasAcceptedOdometrySample) {
                        return true;
                }

                double maxModuleDelta = 0.0;
                for (int i = 0; i < wheelPositions.length; i++) {
                        maxModuleDelta = Math.max(
                                        maxModuleDelta,
                                        Math.abs(wheelPositions[i].distanceMeters
                                                        - lastAcceptedOdometryPositions[i].distanceMeters));
                }
                double yawDelta = Math.abs(yawPosition.minus(lastAcceptedYawPosition).getRadians());
                return maxModuleDelta > DriveConstants.odometryTranslationDeadbandMeters
                                || yawDelta > DriveConstants.odometryYawDeadbandRadians;
        }

        private void seedAcceptedOdometryState(SwerveModulePosition[] wheelPositions, Rotation2d yawPosition) {
                for (int i = 0; i < wheelPositions.length; i++) {
                        lastAcceptedOdometryPositions[i] = new SwerveModulePosition(wheelPositions[i].distanceMeters,
                                        wheelPositions[i].angle);
                }
                lastAcceptedYawPosition = yawPosition;
                hasAcceptedOdometrySample = true;
        }

        private double driveMotorRotationsToMeters(double motorRotations) {
                double wheelRotations = motorRotations / Constants.Ratios.Drive.DRIVE_GEAR_RATIO;
                return wheelRotations / Constants.Physical.WHEEL_ROTATION_PER_METER;
        }

        private SwerveModulePosition[] getCurrentWheelPositions(
                        double frontLeftDrivePosition,
                        double frontRightDrivePosition,
                        double backLeftDrivePosition,
                        double backRightDrivePosition,
                        Rotation2d frontLeftTurnPosition,
                        Rotation2d frontRightTurnPosition,
                        Rotation2d backLeftTurnPosition,
                        Rotation2d backRightTurnPosition) {
                return new SwerveModulePosition[] {
                                new SwerveModulePosition(
                                                driveMotorRotationsToMeters(frontLeftDrivePosition),
                                                frontLeftTurnPosition),
                                new SwerveModulePosition(
                                                driveMotorRotationsToMeters(frontRightDrivePosition),
                                                frontRightTurnPosition),
                                new SwerveModulePosition(
                                                driveMotorRotationsToMeters(backLeftDrivePosition),
                                                backLeftTurnPosition),
                                new SwerveModulePosition(
                                                driveMotorRotationsToMeters(backRightDrivePosition),
                                                backRightTurnPosition)
                };
        }

        private double getAverageTagDistanceMeters(PhotonPipelineResult result) {
                return result.getTargets().stream()
                                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                                .average()
                                .orElse(Double.POSITIVE_INFINITY);
        }

        private Matrix<N3, N1> createPhotonVisionStdDevs(
                        double averageTagDistance,
                        int tagCount,
                        boolean useVisionRotation) {
                double xyStdDev = DriveConstants.photonXyStdDevCoefficient
                                * Math.pow(averageTagDistance, 1.2)
                                / Math.pow(Math.max(tagCount, 1), 2.0);
                double thetaStdDev = useVisionRotation
                                ? DriveConstants.photonThetaStdDevCoefficient
                                                * Math.pow(averageTagDistance, 1.2)
                                                / Math.pow(Math.max(tagCount, 1), 2.0)
                                : Double.POSITIVE_INFINITY;
                Logger.recordOutput("Vision/Photon Std Dev XY", xyStdDev);
                Logger.recordOutput("Vision/Photon Std Dev Theta", thetaStdDev);
                return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
        }

        private Matrix<N3, N1> createVisionStdDevs(double averageTagDistance, int tagCount, double thetaScalar) {
                double xyStdDev = Constants.Vision.getTagDistStdDevScalar(averageTagDistance)
                                * Constants.Vision.getNumTagStdDevScalar(tagCount);
                double thetaStdDev = xyStdDev * thetaScalar;
                Logger.recordOutput("Vision/Limelight Std Dev XY", xyStdDev);
                Logger.recordOutput("Vision/Limelight Std Dev Theta", thetaStdDev);
                return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
        }

        private void addPhotonHeadingData(double timestamp, Rotation2d heading) {
                leftFrontPhotonPoseEstimator.addHeadingData(timestamp, heading);
                leftBackPhotonPoseEstimator.addHeadingData(timestamp, heading);
                rightFrontPhotonPoseEstimator.addHeadingData(timestamp, heading);
                rightBackPhotonPoseEstimator.addHeadingData(timestamp, heading);
        }

        private void resetPhotonHeadingData(double timestamp, Rotation2d heading) {
                if (leftFrontPhotonPoseEstimator == null
                                || leftBackPhotonPoseEstimator == null
                                || rightFrontPhotonPoseEstimator == null
                                || rightBackPhotonPoseEstimator == null) {
                        return;
                }
                leftFrontPhotonPoseEstimator.resetHeadingData(timestamp, heading);
                leftBackPhotonPoseEstimator.resetHeadingData(timestamp, heading);
                rightFrontPhotonPoseEstimator.resetHeadingData(timestamp, heading);
                rightBackPhotonPoseEstimator.resetHeadingData(timestamp, heading);
        }

        private boolean poseInField(Pose2d pose) {

                return pose.getX() > Constants.Physical.ROBOT_RADIUS
                                && pose.getX() < Constants.Physical.FIELD_LENGTH
                                                - Constants.Physical.ROBOT_RADIUS
                                && pose.getY() > Constants.Physical.ROBOT_RADIUS
                                && pose.getY() < Constants.Physical.FIELD_WIDTH
                                                - Constants.Physical.ROBOT_RADIUS;
        }

}
