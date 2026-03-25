package frc.robot.subsystems.drive;

import frc.robot.tools.logging.BatteryLogger;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive.DriveState;
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

        // creates all 4 modules
        private final SwerveModule frontRight = new SwerveModule(1, frontRightAngleMotor, frontRightDriveMotor,
                        frontRightCanCoder);
        private final SwerveModule frontLeft = new SwerveModule(2, frontLeftAngleMotor, frontLeftDriveMotor,
                        frontLeftCanCoder);
        private final SwerveModule backLeft = new SwerveModule(3, backLeftAngleMotor, backLeftDriveMotor,
                        backLeftCanCoder);
        private final SwerveModule backRight = new SwerveModule(4, backRightAngleMotor, backRightDriveMotor,
                        backRightCanCoder);
        PhotonPoseEstimator leftFrontPhotonPoseEstimator;
        PhotonPoseEstimator leftBackPhotonPoseEstimator;
        PhotonPoseEstimator rightFrontPhotonPoseEstimator;
        PhotonPoseEstimator rightBackPhotonPoseEstimator;
        AprilTagFieldLayout aprilTagFieldLayout;

        // *********************NOTE THE PITCH IS POSITIVE DOWNWARDS
        // **********************************

        Transform3d leftFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-9.5614), Constants.inchesToMeters(14.2213),
                                        Constants.inchesToMeters(
                                                        24.1563)),
                        new Rotation3d(Math.toRadians(-0.5), Math.toRadians(-8.0), Math.toRadians(75)));

        Transform3d leftBackRobotToCam = new Transform3d( // front reef cam on swerve module
                        new Translation3d(Constants.inchesToMeters(-11.4675),
                                        Constants.inchesToMeters(13.5008),
                                        Constants.inchesToMeters(24.1563)),
                        new Rotation3d(Math.toRadians(0.7), Math.toRadians(-9.0),
                                        Math.toRadians(145.0)));

        Transform3d rightFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-9.8347), Constants.inchesToMeters(-14.7105),
                                        Constants.inchesToMeters(13.9669)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-26.0), Math.toRadians(275.0)));

        Transform3d rightBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-11.4424), Constants.inchesToMeters(11.4633),
                                        Constants.inchesToMeters(24.1563)),
                        new Rotation3d(Math.toRadians(1.7), Math.toRadians(-11.0),
                                        Math.toRadians(215.0)));

        // xy position of module based on robot width and distance from edge of robot
        private final double moduleX = ((Constants.Physical.ROBOT_LENGTH) / 2) - Constants.Physical.MODULE_OFFSET;
        private final double moduleY = ((Constants.Physical.ROBOT_WIDTH) / 2) - Constants.Physical.MODULE_OFFSET;

        // Locations for the swerve drive modules relative to the robot center.
        private Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
        private Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
        private Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
        private Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

        private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        private SwerveDrivePoseEstimator mt2Odometry;
        @SuppressWarnings("unused")
        private Peripherals peripherals;
        private LinearFilter filterX = LinearFilter.movingAverage(10);
        private LinearFilter filterY = LinearFilter.movingAverage(10);
        private LinearFilter filterOmega = LinearFilter.movingAverage(10);

        private ChassisSpeeds wantedChassisSpeeds = new ChassisSpeeds(0, 0, 0);

        private boolean onBump = false;
        private int numTimesFlat = 0;
        Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
        private Debouncer flatDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kFalling);

        public static TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer
                        .createBuffer(2.0);

        public DriveIOComp(Peripherals peripherals) {

                frontRight.init();
                frontLeft.init();
                backRight.init();
                backLeft.init();
                gyro.init();
                this.peripherals = peripherals;
                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
                swerveModulePositions[0] = new SwerveModulePosition(0,
                                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
                swerveModulePositions[1] = new SwerveModulePosition(0,
                                new Rotation2d(frontRight.getCanCoderPositionRadians()));
                swerveModulePositions[2] = new SwerveModulePosition(0,
                                new Rotation2d(backLeft.getCanCoderPositionRadians()));
                swerveModulePositions[3] = new SwerveModulePosition(0,
                                new Rotation2d(backRight.getCanCoderPositionRadians()));

                Pose2d m_pose = new Pose2d();
                mt2Odometry = new SwerveDrivePoseEstimator(m_kinematics,
                                getYaw(), swerveModulePositions, m_pose);
                try {
                        aprilTagFieldLayout = new AprilTagFieldLayout(
                                        Filesystem.getDeployDirectory().getPath() + "/"
                                                        + "2026-rebuilt.json");
                } catch (Exception e) {
                        java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
                }
                leftFrontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                leftFrontRobotToCam);
                leftBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                leftBackRobotToCam);
                rightFrontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                rightFrontRobotToCam);
                rightBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                rightBackRobotToCam);
        }

        @Override
        void zeroIMU() {
                gyro.setYaw(0.0);
                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
                swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
                                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
                swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
                                new Rotation2d(frontRight.getCanCoderPositionRadians()));
                swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
                                new Rotation2d(backLeft.getCanCoderPositionRadians()));
                swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
                                new Rotation2d(backRight.getCanCoderPositionRadians()));
                mt2Odometry.resetPosition(new Rotation2d(), swerveModulePositions,
                                mt2Odometry.getEstimatedPosition());
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
                wantedChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        @Override
        protected void setCurrentLimits(int supply, int stator) {
                System.out.println("Setting current limit");
                frontLeft.setDriveCurrentLimits(supply, stator);
                frontRight.setDriveCurrentLimits(supply, stator);
                backLeft.setDriveCurrentLimits(supply, stator);
                backRight.setDriveCurrentLimits(supply, stator);
        }

        private boolean notTrenchTag(int tagId) {
                return tagId != 1 &&
                                tagId != 12 &&
                                tagId != 22 &&
                                tagId != 23 &&
                                tagId != 7 &&
                                tagId != 6 &&
                                tagId != 17 &&
                                tagId != 28;
        }

        @Override
        protected void setPosition(Pose2d pose) {
                setYaw(pose.getRotation().getDegrees());
                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
                swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
                                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
                swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
                                new Rotation2d(frontRight.getCanCoderPositionRadians()));
                swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
                                new Rotation2d(backLeft.getCanCoderPositionRadians()));
                swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
                                new Rotation2d(backRight.getCanCoderPositionRadians()));
                mt2Odometry.resetPosition(pose.getRotation(), swerveModulePositions,
                                pose);
        }

        private boolean poseInField(Pose2d pose) {
                return pose.getX() > 0 && pose.getX() < Constants.Physical.FIELD_LENGTH
                                && pose.getY() > 0 && pose.getY() < Constants.Physical.FIELD_WIDTH;
        }

        private Pose2d clampToField(Pose2d pose) {
                if (pose.getX() > (Constants.Physical.FIELD_LENGTH - Constants.Physical.ROBOT_RADIUS)) {
                        pose = new Pose2d(Constants.Physical.FIELD_LENGTH - Constants.Physical.ROBOT_RADIUS,
                                        pose.getY(), pose.getRotation());
                } else if (pose.getX() < (Constants.Physical.ROBOT_RADIUS)) {
                        pose = new Pose2d(Constants.Physical.ROBOT_RADIUS,
                                        pose.getY(), pose.getRotation());
                }

                if (pose.getY() > (Constants.Physical.FIELD_WIDTH - Constants.Physical.ROBOT_RADIUS)) {
                        pose = new Pose2d(pose.getX(),
                                        Constants.Physical.FIELD_WIDTH
                                                        - Constants.Physical.ROBOT_RADIUS,
                                        pose.getRotation());
                } else if (pose.getY() < (Constants.Physical.ROBOT_RADIUS)) {
                        pose = new Pose2d(pose.getX(),
                                        Constants.Physical.ROBOT_RADIUS, pose.getRotation());
                }

                return pose;
        }

        /**
         * Updates the fused odometry array with current robot position and orientation
         * information.
         * Calculates the robot's position and orientation using swerve module positions
         * and the gyro angle.
         * Updates the current X, Y, and theta values, as well as previous values and
         * time differences.
         */
        private void updateOdometryFusedArray(DriveState currentState) {
                SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
                swerveModulePositions[0] = new SwerveModulePosition(frontLeft.getModuleDistance(),
                                new Rotation2d(frontLeft.getCanCoderPositionRadians()));
                swerveModulePositions[1] = new SwerveModulePosition(frontRight.getModuleDistance(),
                                new Rotation2d(frontRight.getCanCoderPositionRadians()));
                swerveModulePositions[2] = new SwerveModulePosition(backLeft.getModuleDistance(),
                                new Rotation2d(backLeft.getCanCoderPositionRadians()));
                swerveModulePositions[3] = new SwerveModulePosition(backRight.getModuleDistance(),
                                new Rotation2d(backRight.getCanCoderPositionRadians()));
                boolean tilted = Math.abs(gyro.getPitchDegrees()) > 4.1 || Math.abs(gyro.getRollDegrees()) > 4.1;
                if (tilted && !onBump) {
                        onBump = true;
                }

                boolean tiltedFiltered = flatDebouncer.calculate(tilted);

                if (!onBump && !tiltedFiltered) {
                        mt2Odometry.update(getYaw(), swerveModulePositions);
                }

                // Logger.recordOutput("Testing/num times flat", numTimesFlat);
                // if (!tiltedFiltered && onBump) {
                // onBump = false;
                // double vx = getChassisSpeeds().vxMetersPerSecond;
                // double direction = Math.signum(vx);
                // Pose2d currentPose = mt2Odometry.getEstimatedPosition();
                // // Logger.recordOutput("Testing/Current pose on bump",
                // // currentPose.getTranslation());
                // Translation2d bump = new Translation2d(
                // direction * Constants.Field.BUMP_LENGTH, 0.0);
                // // Logger.recordOutput("Testing/Bump translation", bump);
                // Pose2d correctedPose = new Pose2d(currentPose.getTranslation().plus(bump),
                // getYaw());
                // // Logger.recordOutput("Testing/Corrected pose", correctedPose);

                // setPosition(correctedPose);
                // }
                // Logger.recordOutput("Testing/on bump", onBump);
                if (!tiltedFiltered && onBump) {
                        onBump = false;

                        Pose2d currentPose = mt2Odometry.getEstimatedPosition();
                        double x = currentPose.getX();
                        double y = currentPose.getY();
                        double vx = getChassisSpeeds().vxMetersPerSecond;
                        double direction = Math.signum(vx);

                        boolean onBlueSide = x < Constants.Physical.FIELD_LENGTH / 2.0;

                        double correctedX;

                        if (onBlueSide) {
                                if (direction == 1) {
                                        correctedX = Constants.Field.NEUTRAL_ZONE_BUMP_X_POSITION_BLUE;
                                } else {
                                        correctedX = Constants.Field.ALLIANCE_ZONE_BUMP_X_POSITION_BLUE;
                                }
                        } else {
                                if (direction == 1) {
                                        correctedX = Constants.Field.ALLIANCE_ZONE_BUMP_X_POSITION_RED;
                                } else {
                                        correctedX = Constants.Field.NEUTRAL_ZONE_BUMP_X_POSITION_RED;
                                }
                        }

                        Pose2d correctedPose = new Pose2d(
                                        new Translation2d(correctedX, y),
                                        getYaw());

                        setPosition(correctedPose);
                }

                addTurretObservation(Timer.getTimestamp(), Globals.turretAngle);

                // Rotation2d robotRotation = new
                // Rotation2d(Math.toRadians(gyro.getYawDegrees()));
                // double time = Timer.getFPGATimestamp();
                // rightFrontPhotonPoseEstimator.addHeadingData(time, robotRotation);
                // rightBackPhotonPoseEstimator.addHeadingData(time, robotRotation);
                // leftFrontPhotonPoseEstimator.addHeadingData(time, robotRotation);

                if (Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) < 2.4) {
                        if (!onBump && !tiltedFiltered) {
                                var rightFrontResult = peripherals.getRightFrontCamResult();
                                Optional<EstimatedRobotPose> rightFrontMultiTagResult = rightFrontPhotonPoseEstimator
                                                .estimateCoprocMultiTagPose(rightFrontResult);
                                if (rightFrontMultiTagResult.isPresent()) {
                                        if (rightFrontResult.getBestTarget().getPoseAmbiguity() < 0.3
                                        // && notTrenchTag(rightFrontResult.getBestTarget().fiducialId)
                                        ) {
                                                standardDeviation.set(0, 0, 1.0);
                                                standardDeviation.set(1, 0, 1.0);
                                                standardDeviation.set(2, 0, 0.7);
                                                Pose2d robotPose = rightFrontMultiTagResult.get().estimatedPose
                                                                .toPose2d();
                                                if (poseInField(robotPose)) {
                                                        Logger.recordOutput("Cameras/Right Front Pose",
                                                                        clampToField(robotPose));
                                                        mt2Odometry.addVisionMeasurement(clampToField(robotPose),
                                                                        rightFrontResult.getTimestampSeconds(),
                                                                        standardDeviation);
                                                } else {
                                                        Logger.recordOutput("Cameras/Right Front Pose",
                                                                        new Pose2d());
                                                }
                                        }
                                }
                                if (currentState != DriveState.DRIVE_TO_ALIGN_CLIMB
                                                && currentState != DriveState.DRIVE_TO_PRE_CLIMB) {
                                        var rightBackResult = peripherals.getRightBackCamResult();
                                        Optional<EstimatedRobotPose> rightBackMultiTagResult = rightBackPhotonPoseEstimator
                                                        .estimateCoprocMultiTagPose(rightBackResult);
                                        if (rightBackMultiTagResult.isPresent()) {
                                                if (rightBackResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                // && notTrenchTag(rightBackResult
                                                // .getBestTarget().fiducialId)
                                                ) {
                                                        standardDeviation.set(0, 0, 1.0);
                                                        standardDeviation.set(1, 0, 1.0);
                                                        standardDeviation.set(2, 0, 0.9);
                                                        Pose2d robotPose = rightBackMultiTagResult.get().estimatedPose
                                                                        .toPose2d();
                                                        if (poseInField(robotPose)) {
                                                                Logger.recordOutput("Cameras/Right Back Pose",
                                                                                clampToField(robotPose));
                                                                mt2Odometry.addVisionMeasurement(
                                                                                clampToField(robotPose),
                                                                                rightFrontResult.getTimestampSeconds(),
                                                                                standardDeviation);
                                                        } else {
                                                                Logger.recordOutput("Cameras/Right Back Pose",
                                                                                new Pose2d());
                                                        }
                                                }
                                        }
                                        var leftBackResult = peripherals.getLeftBackCamResult();
                                        Optional<EstimatedRobotPose> leftBackMultiTagResult = leftBackPhotonPoseEstimator
                                                        .estimateCoprocMultiTagPose(leftBackResult);
                                        if (leftBackMultiTagResult.isPresent()) {
                                                if (leftBackResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                // && notTrenchTag(leftBackResult
                                                // .getBestTarget().fiducialId)
                                                ) {
                                                        standardDeviation.set(0, 0, 1.3);
                                                        standardDeviation.set(1, 0, 1.3);
                                                        standardDeviation.set(2, 0, 0.9);
                                                        Pose2d robotPose = leftBackMultiTagResult.get().estimatedPose
                                                                        .toPose2d();
                                                        if (poseInField(robotPose)) {
                                                                Logger.recordOutput("Cameras/Left Back Pose",
                                                                                clampToField(robotPose));
                                                                mt2Odometry.addVisionMeasurement(
                                                                                clampToField(robotPose),
                                                                                leftBackResult.getTimestampSeconds(),
                                                                                standardDeviation);
                                                        } else {
                                                                Logger.recordOutput("Cameras/Left Back Pose",
                                                                                new Pose2d());
                                                        }
                                                }
                                        }

                                        var leftFrontResult = peripherals.getLeftFrontCamResult();
                                        Optional<EstimatedRobotPose> leftFrontMultiTagResult = leftFrontPhotonPoseEstimator
                                                        .estimateCoprocMultiTagPose(leftFrontResult);
                                        if (leftFrontMultiTagResult.isPresent()) {
                                                if (leftFrontResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                // && notTrenchTag(leftFrontResult
                                                // .getBestTarget().fiducialId)
                                                ) {
                                                        standardDeviation.set(0, 0, 1.0);
                                                        standardDeviation.set(1, 0, 1.0);
                                                        standardDeviation.set(2, 0, 0.9);
                                                        Pose2d robotPose = leftFrontMultiTagResult.get().estimatedPose
                                                                        .toPose2d();
                                                        if (poseInField(robotPose)) {
                                                                Logger.recordOutput("Cameras/Left Front Pose",
                                                                                clampToField(robotPose));
                                                                mt2Odometry.addVisionMeasurement(
                                                                                clampToField(robotPose),
                                                                                leftFrontResult.getTimestampSeconds(),
                                                                                standardDeviation);
                                                        } else {
                                                                Logger.recordOutput("Cameras/Left Front Pose",
                                                                                new Pose2d());
                                                        }
                                                }
                                        }

                                        double limelightAngVelRelToField = Constants.Vision
                                                        .getLimelightAngVelRelToField(
                                                                        Globals.turretVelocity,
                                                                        getChassisSpeeds().omegaRadiansPerSecond);
                                        Logger.recordOutput("Limelight Ang Vel", limelightAngVelRelToField);
                                        if (Math.abs(limelightAngVelRelToField) < 0.5) {
                                                try {
                                                        LimelightHelpers.SetRobotOrientation(
                                                                        Constants.Vision.LIMELIGHT_NAME,
                                                                        getPosition().getRotation().getDegrees(),
                                                                        limelightAngVelRelToField,
                                                                        gyro.getPitchDegrees(), 0,
                                                                        -gyro.getRollDegrees(), 0);
                                                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                                                        .getBotPoseEstimate_wpiBlue_MegaTag2(
                                                                                        Constants.Vision.LIMELIGHT_NAME);
                                                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                                                        .getBotPoseEstimate_wpiBlue(
                                                                                        Constants.Vision.LIMELIGHT_NAME);
                                                        Logger.recordOutput("Cameras/Limelight Pose MT1", mt1.pose);

                                                        // Optional<Rotation2d> maybeTurretAngle = getTurretAngle(
                                                        // mt2.timestampSeconds);
                                                        // if (maybeTurretAngle.isPresent()) {
                                                        Constants.Vision.updateLimelightPoseFromTurret(
                                                                        new Pose3d(Constants.Physical.Shooter.SHOOTER_POSITION,
                                                                                        Rotation3d.kZero),
                                                                        Globals.turretAngle,
                                                                        Constants.Vision.turretToLimelight,
                                                                        Constants.Vision.LIMELIGHT_NAME);

                                                        boolean doRejectUpdate = false;
                                                        // if (Math.abs(gyro.getAngularVelocityZDeviceDegPerSec()) >
                                                        // 360) {
                                                        // doRejectUpdate = true;
                                                        // }
                                                        if (mt2.tagCount == 0) {
                                                                doRejectUpdate = true;
                                                        }
                                                        if (!doRejectUpdate) {
                                                                Logger.recordOutput("Limelight dist to tag",
                                                                                mt2.avgTagDist);
                                                                standardDeviation.set(0, 0, 2.1);
                                                                standardDeviation.set(1, 0, 2.1);
                                                                standardDeviation.set(2, 0, 5.0);
                                                                // if (mt2.avgTagDist < 4.5) {
                                                                Logger.recordOutput("Cameras/Limelight Pose",
                                                                                clampToField(mt2.pose));
                                                                mt2Odometry.addVisionMeasurement(
                                                                                clampToField(mt2.pose),
                                                                                mt2.timestampSeconds,
                                                                                standardDeviation);
                                                                // }
                                                        } else {
                                                                Logger.recordOutput("Cameras/Limelight Pose",
                                                                                new Pose2d());
                                                        }
                                                        // } else {
                                                        // System.out.println("Turret angle not found for timestamp: "
                                                        // + mt2.timestampSeconds);
                                                        // }
                                                } catch (Exception e) {
                                                        System.out.println(e);
                                                }
                                        }
                                }
                        }

                }

                // Module states
                var frontLeftState = frontLeft.getSwerveModuleState(gyro.getYaw());
                var frontRightState = frontRight.getSwerveModuleState(gyro.getYaw());
                var backLeftState = backLeft.getSwerveModuleState(gyro.getYaw());
                var backRightState = backRight.getSwerveModuleState(gyro.getYaw());
                // Convert to chassis speeds
                ChassisSpeeds robotSpeeds = m_kinematics.toChassisSpeeds(
                                frontLeftState, frontRightState, backLeftState, backRightState);
                filterX.calculate(robotSpeeds.vxMetersPerSecond);
                filterY.calculate(robotSpeeds.vyMetersPerSecond);
                filterOmega.calculate(Math.toRadians(gyro.getAngularVelocityZWorldDegPerSec()));

        }

        public Optional<Rotation2d> getTurretAngle(double timestamp) {
                return turretAngleBuffer.getSample(timestamp);
        }

        public void addTurretObservation(double timestamp, Rotation2d turretAngle) {
                turretAngleBuffer.addSample(timestamp, turretAngle);
        }

        @Override
        protected Pose2d getPosition() {
                // double x = mt2Odometry.getEstimatedPosition().getX();
                // double y = mt2Odometry.getEstimatedPosition().getY();
                // Rotation2d heading = gyro.getYaw();
                // return new Pose2d(x, y, heading);
                return mt2Odometry.getEstimatedPosition();
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
                frontLeft.drive(velocityVector, turnRadiansPerSec, 0);
                frontRight.drive(velocityVector, turnRadiansPerSec, 0);
                backLeft.drive(velocityVector, turnRadiansPerSec, 0);
                backRight.drive(velocityVector, turnRadiansPerSec, 0);
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
                ChassisSpeeds avg = new ChassisSpeeds(filterX.lastValue(), filterY.lastValue(),
                                filterOmega.lastValue());
                // Logger.recordOutput("Drive/RobotVelocities/X", avg.vxMetersPerSecond);
                // Logger.recordOutput("Drive/RobotVelocities/Y", avg.vyMetersPerSecond);
                // Logger.recordOutput("Drive/RobotVelocities/Omega",
                // avg.omegaRadiansPerSecond);
                return avg;
        }

        @Override
        void update(DriveState currentState) {
                updateOdometryFusedArray(currentState);
                getChassisSpeeds();

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
                // Logger.recordOutput("Robot/turret velocity filtered",
                // Globals.turretVelocity);
                // Logger.recordOutput("Robot/limelight ang vel rel to turret",
                // Constants.Vision.getLimelightAngVelRelToField(Globals.turretVelocity,
                // getChassisSpeeds().omegaRadiansPerSecond));
                // Logger.recordOutput("Robot/chassis speeds ang vel",
                // getChassisSpeeds().omegaRadiansPerSecond);
                Logger.recordOutput("Robot/pitch", gyro.getPitchDegrees());
                Logger.recordOutput("Robot/roll", gyro.getRollDegrees());

        }

        @Override
        protected ChassisSpeeds getWantedChassisSpeeds() {
                return wantedChassisSpeeds;
        }

        @Override
        protected boolean getFlat() {
                return Math.abs(gyro.getPitchDegrees()) < 3.5 && Math.abs(gyro.getRollDegrees()) < 3.5;
        }
}
