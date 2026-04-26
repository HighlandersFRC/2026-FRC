package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;

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
        private static final int ODOMETRY_QUEUE_CAPACITY = 50;
        private static final Pose2d ZERO_POSE = Pose2d.kZero;
        private static final Comparator<PendingVisionObservation> VISION_OBSERVATION_TIMESTAMP_COMPARATOR = Comparator
                        .comparingDouble(PendingVisionObservation::timestamp);

        private final Set<Integer> filteredPhotonTagIds = Set.of(15, 16, 31, 32);

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

        private final Transform3d leftFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-13.3), Constants.inchesToMeters(7.17),
                                        Constants.inchesToMeters(23.79)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-9.8), Math.toRadians(78)));

        private final Transform3d leftBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-14.206),
                                        Constants.inchesToMeters(7.265),
                                        Constants.inchesToMeters(22.265)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-9.0),
                                        Math.toRadians(145.0)));

        private final Transform3d rightFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-13.672), Constants.inchesToMeters(-7.257),
                                        Constants.inchesToMeters(23.933)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-9.9), Math.toRadians(282.0)));

        private final Transform3d rightBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-14.213), Constants.inchesToMeters(3.807),
                                        Constants.inchesToMeters(23.057)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-10.0),
                                        Math.toRadians(210.0)));

        private final LinearFilter filterX = LinearFilter.movingAverage(10);
        private final LinearFilter filterY = LinearFilter.movingAverage(10);
        private final LinearFilter filterOmega = LinearFilter.movingAverage(10);
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
        private final double[] timestampSamples = new double[ODOMETRY_QUEUE_CAPACITY];
        private final double[] frontLeftDriveSamples = new double[ODOMETRY_QUEUE_CAPACITY];
        private final double[] frontRightDriveSamples = new double[ODOMETRY_QUEUE_CAPACITY];
        private final double[] backLeftDriveSamples = new double[ODOMETRY_QUEUE_CAPACITY];
        private final double[] backRightDriveSamples = new double[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] frontLeftTurnSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] frontRightTurnSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] backLeftTurnSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] backRightTurnSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] yawSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] pitchSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final Rotation2d[] rollSamples = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];
        private final SwerveModulePosition[] odometryWheelPositionScratch = new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
        };
        private final List<PendingVisionObservation> pendingVisionObservations = new ArrayList<>(16);
        private final List<Pose2d> acceptedPhotonPoses = new ArrayList<>(16);

        private ChassisSpeeds wantedChassisSpeeds = new ChassisSpeeds();

        private record PendingVisionObservation(
                        String logKey,
                        Pose2d pose,
                        double timestamp,
                        Matrix<N3, N1> stdDevs) {
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
                        AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(
                                        Filesystem.getDeployDirectory().getPath() + "/" + "2026-rebuilt.json");
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
                gyro.setPitchOffsetDegrees(gyro.getPitchDegrees());
                gyro.setRollOffsetDegrees(gyro.getRollDegrees());
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
                Logger.recordOutput("Auto/FirstPose", pose);
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
                        RobotState.getInstance().resetPose(pose, currentWheelPositions,
                                        Optional.of(pose.getRotation()));
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
                double cosYaw = Math.cos(yaw);
                double sinYaw = Math.sin(yaw);
                double fieldRelativeX = velocityVector.getI() * cosYaw - velocityVector.getJ() * sinYaw;
                double fieldRelativeY = velocityVector.getI() * sinYaw + velocityVector.getJ() * cosYaw;
                wantedChassisSpeeds = new ChassisSpeeds(
                                fieldRelativeX,
                                fieldRelativeY,
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
                double cosCam = Math.cos(camAngle);
                double sinCam = Math.sin(camAngle);
                double fieldRelativeX = velocityVector.getI() * cosCam - velocityVector.getJ() * sinCam;
                double fieldRelativeY = velocityVector.getI() * sinCam + velocityVector.getJ() * cosCam;
                wantedChassisSpeeds = new ChassisSpeeds(
                                fieldRelativeX,
                                fieldRelativeY,
                                turnRadiansPerSec);
        }

        @Override
        protected ChassisSpeeds getChassisSpeeds() {
                return new ChassisSpeeds(filterX.lastValue(), filterY.lastValue(), filterOmega.lastValue());
        }

        @Override
        void update(DriveState currentState) {
                updateOdometry();
                updateMeasuredChassisSpeeds();
                updateVision(currentState);

                // batteryLogger.reportCurrentUsage("Drive/FrontRight",
                // frontRight.getDriveMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/FrontRightTurn",
                // frontRight.getAngleMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/FrontLeft",
                // frontLeft.getDriveMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/FrontLeftTurn",
                // frontLeft.getAngleMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/BackRight",
                // backRight.getDriveMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/BackRightTurn",
                // backRight.getAngleMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/BackLeft",
                // backLeft.getDriveMotorSupplyCurrent());
                // batteryLogger.reportCurrentUsage("Drive/BackLeftTurn",
                // backLeft.getAngleMotorSupplyCurrent());

                // Logger.recordOutput("Swerve/Front Right Drive Current",
                // frontRight.getDriveMotorCurrent());
                // Logger.recordOutput("Swerve/Front Left Drive Current",
                // frontLeft.getDriveMotorCurrent());
                // Logger.recordOutput("Swerve/Back Right Drive Current",
                // backRight.getDriveMotorCurrent());
                // Logger.recordOutput("Swerve/Back Left Drive Current",
                // backLeft.getDriveMotorCurrent());
                // Logger.recordOutput("Swerve/Front Right Angle Current",
                // frontRight.getAngleMotorCurrent());
                // Logger.recordOutput("Swerve/Front Left Angle Current",
                // frontLeft.getAngleMotorCurrent());
                // Logger.recordOutput("Swerve/Back Right Angle Current",
                // backRight.getAngleMotorCurrent());
                // Logger.recordOutput("Swerve/Back Left Angle Current",
                // backLeft.getAngleMotorCurrent());
                Logger.recordOutput("Robot/pitch", gyro.getPitchDegrees());
                Logger.recordOutput("Robot/roll", gyro.getRollDegrees());

                Logger.recordOutput("Robot/gyro1yaw", gyro.getPigeon1Yaw());
                Logger.recordOutput("Robot/gyro2yaw", gyro.getPigeon2Yaw());
                Logger.recordOutput("Robot/gyro1roll", gyro.getPigeon1Roll());
                Logger.recordOutput("Robot/gyro2roll", gyro.getPigeon2Roll());
                Logger.recordOutput("Robot/gyro1pitch", gyro.getPigeon1Pitch());
                Logger.recordOutput("Robot/gyro2pitch", gyro.getPigeon2Pitch());

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
                Logger.recordOutput("Pigeon2 Online", gyro.is2Online());
        }

        @Override
        protected ChassisSpeeds getWantedChassisSpeeds() {
                return new ChassisSpeeds(wantedChassisSpeeds.vxMetersPerSecond, wantedChassisSpeeds.vyMetersPerSecond,
                                -2.5 * wantedChassisSpeeds.omegaRadiansPerSecond);
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
                int sampleCount;
                Drive.odometryLock.lock();
                try {
                        sampleCount = drainOdometryQueuesToSamples();
                } finally {
                        Drive.odometryLock.unlock();
                }

                int acceptedSamples = 0;

                for (int i = 0; i < sampleCount; i++) {
                        populateWheelPositions(
                                        odometryWheelPositionScratch,
                                        frontLeftDriveSamples[i],
                                        frontRightDriveSamples[i],
                                        backLeftDriveSamples[i],
                                        backRightDriveSamples[i],
                                        frontLeftTurnSamples[i],
                                        frontRightTurnSamples[i],
                                        backLeftTurnSamples[i],
                                        backRightTurnSamples[i]);
                        Rotation2d yawPosition = yawSamples[i];
                        if (!shouldAcceptOdometrySample(odometryWheelPositionScratch, yawPosition)) {
                                continue;
                        }

                        RobotState.getInstance().addOdometryObservation(new RobotState.OdometryObservation(
                                        timestampSamples[i],
                                        odometryWheelPositionScratch,
                                        Optional.of(rollSamples[i]),
                                        Optional.of(pitchSamples[i]),
                                        Optional.of(yawPosition)));
                        acceptedSamples++;
                }

                Logger.recordOutput("Drive/OdometryQueuedSamples", sampleCount);
                Logger.recordOutput("Drive/OdometryAcceptedSamples", acceptedSamples);
        }

        private void updateMeasuredChassisSpeeds() {
                Rotation2d yaw = getYaw();
                ChassisSpeeds robotSpeeds = kinematics.toChassisSpeeds(
                                frontLeft.getSwerveModuleState(yaw),
                                frontRight.getSwerveModuleState(yaw),
                                backLeft.getSwerveModuleState(yaw),
                                backRight.getSwerveModuleState(yaw));

                filterX.calculate(robotSpeeds.vxMetersPerSecond);
                filterY.calculate(robotSpeeds.vyMetersPerSecond);
                filterOmega.calculate(Math.toRadians(gyro.getAngularVelocityZWorldDegPerSec()));
        }

        private void updateVision(DriveState currentState) {
                if (leftFrontPhotonPoseEstimator == null
                                || leftBackPhotonPoseEstimator == null
                                || rightFrontPhotonPoseEstimator == null
                                || rightBackPhotonPoseEstimator == null) {
                        return;
                }

                // boolean tilted = Math.abs(gyro.getPitchDegrees()) > 4.1 ||
                // Math.abs(gyro.getRollDegrees()) > 4.1;
                // if (tilted) {
                // return;
                // }

                if (Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) >= 2.4) {
                        return;
                }

                addPhotonHeadingData(Timer.getFPGATimestamp(), RobotState.getInstance().getRotation());

                pendingVisionObservations.clear();
                acceptedPhotonPoses.clear();

                processPhotonResults(
                                "Cameras/Right Front Pose",
                                peripherals.getRightFrontCamResults(),
                                rightFrontPhotonPoseEstimator,
                                pendingVisionObservations,
                                acceptedPhotonPoses);

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

                pendingVisionObservations.sort(VISION_OBSERVATION_TIMESTAMP_COMPARATOR);
                for (PendingVisionObservation observation : pendingVisionObservations) {
                        RobotState.getInstance().addVisionObservation(
                                        new RobotState.VisionObservation(
                                                        observation.timestamp(),
                                                        new Pose3d(observation.pose()),
                                                        observation.stdDevs()));
                }

                Logger.recordOutput("Cameras/Photon Accepted Poses", acceptedPhotonPoses.toArray(Pose2d[]::new));
                Logger.recordOutput("Cameras/Vision Observation Count", pendingVisionObservations.size());
        }

        private void processPhotonResults(
                        String logKey,
                        List<PhotonPipelineResult> results,
                        PhotonPoseEstimator estimator,
                        List<PendingVisionObservation> pendingVisionObservations,
                        List<Pose2d> acceptedPhotonPoses) {
                Pose2d loggedPose = ZERO_POSE;
                for (PhotonPipelineResult result : results) {
                        Optional<PendingVisionObservation> observation = createPhotonObservation(logKey, estimator,
                                        result);
                        if (observation.isPresent()) {
                                PendingVisionObservation acceptedObservation = observation.get();
                                pendingVisionObservations.add(acceptedObservation);
                                acceptedPhotonPoses.add(acceptedObservation.pose());
                                loggedPose = acceptedObservation.pose();
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
                int[] seenTagIds = getPhotonTagIds(result);
                Logger.recordOutput(logKey + "/SeenTagIds", seenTagIds);
                if (containsFilteredPhotonTag(seenTagIds)) {
                        Logger.recordOutput(logKey + "/RejectedTagIds", seenTagIds);
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

                double timestamp = result.getTimestampSeconds();
                Logger.recordOutput(logKey + "/LatencySecs", Timer.getFPGATimestamp() - timestamp);
                return Optional.of(
                                new PendingVisionObservation(
                                                logKey,
                                                robotPose,
                                                timestamp,
                                                createPhotonVisionStdDevs(
                                                                getAverageTagDistanceMeters(result),
                                                                result.getTargets().size(),
                                                                estimate.get().strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)));
        }

        private int[] getPhotonTagIds(PhotonPipelineResult result) {
                int targetCount = result.getTargets().size();
                if (targetCount == 0) {
                        return new int[0];
                }
                int[] seenTagIds = new int[targetCount];
                int seenCount = 0;
                for (int i = 0; i < targetCount; i++) {
                        int tagId = result.getTargets().get(i).getFiducialId();
                        if (tagId <= 0) {
                                continue;
                        }
                        boolean duplicate = false;
                        for (int j = 0; j < seenCount; j++) {
                                if (seenTagIds[j] == tagId) {
                                        duplicate = true;
                                        break;
                                }
                        }
                        if (!duplicate) {
                                seenTagIds[seenCount] = tagId;
                                seenCount++;
                        }
                }
                Arrays.sort(seenTagIds, 0, seenCount);
                return Arrays.copyOf(seenTagIds, seenCount);
        }

        private boolean containsFilteredPhotonTag(int[] seenTagIds) {
                for (int tagId : seenTagIds) {
                        if (filteredPhotonTagIds.contains(tagId)) {
                                return true;
                        }
                }
                return false;
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

        private void updateLimelightObservation(List<PendingVisionObservation> pendingVisionObservations) {
                double maxAcceptedLimelightAngularVelocityRadPerSec = 0.5;
                double limelightAngularVelocityStdDevScalarCoefficient = 2;
                double limelightAngVelRelToField = Constants.Vision.getLimelightAngVelRelToField(
                                Globals.turretVelocity,
                                getChassisSpeeds().omegaRadiansPerSecond);
                Logger.recordOutput("Limelight Ang Vel", limelightAngVelRelToField);
                if (Math.abs(limelightAngVelRelToField) >= maxAcceptedLimelightAngularVelocityRadPerSec) {
                        return;
                }

                try {
                        Rotation2d currentTurretAngle = Globals.turretAngle;
                        Constants.Vision.updateLimelightPoseFromTurret(
                                        new Pose3d(Constants.Physical.Shooter.SHOOTER_POSITION, Rotation3d.kZero),
                                        currentTurretAngle,
                                        Constants.Vision.turretToLimelight,
                                        Constants.Vision.LIMELIGHT_NAME);

                        LimelightHelpers.SetRobotOrientation(
                                        Constants.Vision.LIMELIGHT_NAME,
                                        getPosition().getRotation().getDegrees(),
                                        limelightAngVelRelToField,
                                        gyro.getPitchDegrees(),
                                        0.0,
                                        -gyro.getRollDegrees(),
                                        0.0);

                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue(Constants.Vision.LIMELIGHT_NAME);
                        Logger.recordOutput("Cameras/Limelight Pose MT1", mt1 != null ? mt1.pose : ZERO_POSE);

                        if (mt1 == null) {
                                Logger.recordOutput("Cameras/Limelight Pose", ZERO_POSE);
                                return;
                        }

                        if (mt1.timestampSeconds <= 0.0) {
                                Logger.recordOutput("Cameras/Limelight Pose", ZERO_POSE);
                                return;
                        }

                        if (mt1.tagCount == 0 || mt1.avgTagDist > 4.5 || !poseInField(mt1.pose)) {
                                Logger.recordOutput("Cameras/Limelight Pose", ZERO_POSE);
                                return;
                        }

                        Logger.recordOutput("Limelight dist to tag", mt1.avgTagDist);
                        Logger.recordOutput("Cameras/Limelight Pose", mt1.pose);
                        pendingVisionObservations.add(new PendingVisionObservation(
                                        "Cameras/Limelight Pose",
                                        mt1.pose,
                                        mt1.timestampSeconds,
                                        createVisionStdDevs(mt1.avgTagDist, mt1.tagCount, 2.0,
                                                        limelightAngVelRelToField,
                                                        limelightAngularVelocityStdDevScalarCoefficient)));
                } catch (Exception e) {
                        System.out.println(e);
                }
        }

        private boolean shouldAcceptOdometrySample(SwerveModulePosition[] wheelPositions, Rotation2d yawPosition) {
                return true;
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
                SwerveModulePosition[] wheelPositions = new SwerveModulePosition[] {
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition()
                };
                populateWheelPositions(
                                wheelPositions,
                                frontLeftDrivePosition,
                                frontRightDrivePosition,
                                backLeftDrivePosition,
                                backRightDrivePosition,
                                frontLeftTurnPosition,
                                frontRightTurnPosition,
                                backLeftTurnPosition,
                                backRightTurnPosition);
                return wheelPositions;
        }

        private double getAverageTagDistanceMeters(PhotonPipelineResult result) {
                int targetCount = result.getTargets().size();
                if (targetCount == 0) {
                        return Double.POSITIVE_INFINITY;
                }
                double sumDistance = 0.0;
                for (int i = 0; i < targetCount; i++) {
                        sumDistance += result.getTargets().get(i).getBestCameraToTarget().getTranslation().getNorm();
                }
                return sumDistance / targetCount;
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

        private Matrix<N3, N1> createVisionStdDevs(
                        double averageTagDistance,
                        int tagCount,
                        double thetaScalar,
                        double limelightAngVelRelToField,
                        double limelightAngularVelocityStdDevScalarCoefficient) {
                double angularVelocityStdDevScalar = 1.0
                                + limelightAngularVelocityStdDevScalarCoefficient
                                                * Math.abs(limelightAngVelRelToField);
                double xyStdDev = DriveConstants.photonXyStdDevCoefficient
                                * Math.pow(averageTagDistance, 1.2)
                                / Math.pow(Math.max(tagCount, 1), 2.0)
                                * angularVelocityStdDevScalar;
                double thetaStdDev = Double.POSITIVE_INFINITY;
                Logger.recordOutput("Vision/Limelight Ang Vel Std Dev Scalar", angularVelocityStdDevScalar);
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

        private int drainOdometryQueuesToSamples() {
                int sampleCount = timestampQueue.size();
                sampleCount = Math.min(sampleCount, frontLeftDrivePositionQueue.size());
                sampleCount = Math.min(sampleCount, frontRightDrivePositionQueue.size());
                sampleCount = Math.min(sampleCount, backLeftDrivePositionQueue.size());
                sampleCount = Math.min(sampleCount, backRightDrivePositionQueue.size());
                sampleCount = Math.min(sampleCount, frontLeftTurnPositionQueue.size());
                sampleCount = Math.min(sampleCount, frontRightTurnPositionQueue.size());
                sampleCount = Math.min(sampleCount, backLeftTurnPositionQueue.size());
                sampleCount = Math.min(sampleCount, backRightTurnPositionQueue.size());
                sampleCount = Math.min(sampleCount, yawPositionQueue.size());
                sampleCount = Math.min(sampleCount, pitchPositionQueue.size());
                sampleCount = Math.min(sampleCount, rollPositionQueue.size());
                sampleCount = Math.min(sampleCount, ODOMETRY_QUEUE_CAPACITY);

                for (int i = 0; i < sampleCount; i++) {
                        timestampSamples[i] = pollQueueValue(timestampQueue);
                        frontLeftDriveSamples[i] = pollQueueValue(frontLeftDrivePositionQueue);
                        frontRightDriveSamples[i] = pollQueueValue(frontRightDrivePositionQueue);
                        backLeftDriveSamples[i] = pollQueueValue(backLeftDrivePositionQueue);
                        backRightDriveSamples[i] = pollQueueValue(backRightDrivePositionQueue);
                        frontLeftTurnSamples[i] = Rotation2d.fromRotations(pollQueueValue(frontLeftTurnPositionQueue));
                        frontRightTurnSamples[i] = Rotation2d
                                        .fromRotations(pollQueueValue(frontRightTurnPositionQueue));
                        backLeftTurnSamples[i] = Rotation2d.fromRotations(pollQueueValue(backLeftTurnPositionQueue));
                        backRightTurnSamples[i] = Rotation2d.fromRotations(pollQueueValue(backRightTurnPositionQueue));
                        yawSamples[i] = Rotation2d.fromDegrees(pollQueueValue(yawPositionQueue));
                        pitchSamples[i] = Rotation2d.fromDegrees(pollQueueValue(pitchPositionQueue));
                        rollSamples[i] = Rotation2d.fromDegrees(pollQueueValue(rollPositionQueue));
                }

                clearOdometryQueues();
                return sampleCount;
        }

        private static double pollQueueValue(Queue<Double> queue) {
                Double value = queue.poll();
                return value != null ? value.doubleValue() : 0.0;
        }

        private void populateWheelPositions(
                        SwerveModulePosition[] wheelPositions,
                        double frontLeftDrivePosition,
                        double frontRightDrivePosition,
                        double backLeftDrivePosition,
                        double backRightDrivePosition,
                        Rotation2d frontLeftTurnPosition,
                        Rotation2d frontRightTurnPosition,
                        Rotation2d backLeftTurnPosition,
                        Rotation2d backRightTurnPosition) {
                wheelPositions[0].distanceMeters = driveMotorRotationsToMeters(frontLeftDrivePosition);
                wheelPositions[0].angle = frontLeftTurnPosition;
                wheelPositions[1].distanceMeters = driveMotorRotationsToMeters(frontRightDrivePosition);
                wheelPositions[1].angle = frontRightTurnPosition;
                wheelPositions[2].distanceMeters = driveMotorRotationsToMeters(backLeftDrivePosition);
                wheelPositions[2].angle = backLeftTurnPosition;
                wheelPositions[3].distanceMeters = driveMotorRotationsToMeters(backRightDrivePosition);
                wheelPositions[3].angle = backRightTurnPosition;
        }

}
