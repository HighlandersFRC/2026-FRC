package frc.robot.subsystems.drive;

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
                        new Translation3d(Constants.inchesToMeters(3.125), Constants.inchesToMeters(14.38),
                                        Constants.inchesToMeters(23.75)),
                        new Rotation3d(Math.toRadians(0.5), Math.toRadians(-4.3), Math.toRadians(126.3)));

        Transform3d leftBackRobotToCam = new Transform3d( // front reef cam on swerve module
                        new Translation3d(Constants.inchesToMeters(-10.25),
                                        Constants.inchesToMeters(14.5),
                                        Constants.inchesToMeters(23.5)),
                        new Rotation3d(Math.toRadians(4.0), Math.toRadians(-5.4),
                                        Math.toRadians(54.8)));

        Transform3d rightFrontRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-9.6521), Constants.inchesToMeters(-14.6780),
                                        Constants.inchesToMeters(13.8457)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-27.0), Math.toRadians(-65.0)));

        Transform3d rightBackRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-10.1431), Constants.inchesToMeters(-14.765415),
                                        Constants.inchesToMeters(15.879)),
                        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-26.0),
                                        Math.toRadians(-110.0)));

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
                                PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, leftFrontRobotToCam);
                leftBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftBackRobotToCam);
                rightFrontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, rightFrontRobotToCam);
                rightBackPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, rightBackRobotToCam);
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
                boolean tilted = Math.abs(gyro.getPitchDegrees()) > 5.0 || Math.abs(gyro.getRollDegrees()) > 5.0;
                if (tilted && !onBump) {
                        onBump = true;
                }

                boolean tiltedFiltered = flatDebouncer.calculate(tilted);

                if (!onBump && !tiltedFiltered) {
                        mt2Odometry.update(getYaw(), swerveModulePositions);
                }

                Logger.recordOutput("num times flat", numTimesFlat);
                if (!tiltedFiltered && onBump) {
                        onBump = false;
                        double vx = getChassisSpeeds().vxMetersPerSecond;
                        double direction = Math.signum(vx);
                        Pose2d currentPose = mt2Odometry.getEstimatedPosition();
                        Logger.recordOutput("Current pose on bump", currentPose.getTranslation());
                        Translation2d bump = new Translation2d(
                                        direction * Constants.Field.BUMP_LENGTH, 0.0);
                        Logger.recordOutput("Bump translation", bump);
                        Pose2d correctedPose = new Pose2d(currentPose.getTranslation().plus(bump),
                                        currentPose.getRotation());
                        Logger.recordOutput("Corrected pose", correctedPose.getTranslation());
                        setPosition(correctedPose);
                }
                Logger.recordOutput("on bump", onBump);

                addTurretObservation(Timer.getTimestamp(), Globals.turretAngle);

                Rotation2d robotRotation = new Rotation2d(Math.toRadians(gyro.getYawDegrees()));
                double time = Timer.getFPGATimestamp();
                rightFrontPhotonPoseEstimator.addHeadingData(time, robotRotation);
                rightBackPhotonPoseEstimator.addHeadingData(time, robotRotation);
                leftFrontPhotonPoseEstimator.addHeadingData(time, robotRotation);

                if (Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) < 2.4) {
                        var rightFrontResult = peripherals.getRightFrontCamResult();
                        Optional<EstimatedRobotPose> rightFrontMultiTagResult = rightFrontPhotonPoseEstimator
                                        .update(rightFrontResult);
                        if (rightFrontMultiTagResult.isPresent()) {
                                if (rightFrontResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                                        standardDeviation.set(0, 0, 0.7);
                                        standardDeviation.set(1, 0, 0.7);
                                        standardDeviation.set(2, 0, 2.5);
                                        Pose3d robotPose = rightFrontMultiTagResult.get().estimatedPose;
                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                        rightFrontResult.getTimestampSeconds(),
                                                        standardDeviation);
                                }
                        }

                        var rightBackResult = peripherals.getRightBackCamResult();
                        Optional<EstimatedRobotPose> rightBackMultiTagResult = rightBackPhotonPoseEstimator
                                        .update(rightBackResult);
                        if (rightBackMultiTagResult.isPresent()) {
                                if (rightBackResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                                        standardDeviation.set(0, 0, 0.7);
                                        standardDeviation.set(1, 0, 0.7);
                                        standardDeviation.set(2, 0, 2.5);
                                        Pose3d robotPose = rightBackMultiTagResult.get().estimatedPose;
                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                        rightBackResult.getTimestampSeconds(),
                                                        standardDeviation);
                                }
                        }
                        if (currentState != DriveState.DRIVE_TO_ALIGN_CLIMB
                                        && currentState != DriveState.DRIVE_TO_PRE_CLIMB) {
                                var leftBackResult = peripherals.getLeftBackCamResult();
                                Optional<EstimatedRobotPose> leftBackMultiTagResult = leftBackPhotonPoseEstimator
                                                .update(leftBackResult);
                                if (leftBackMultiTagResult.isPresent()) {
                                        if (leftBackResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                                                standardDeviation.set(0, 0, 1.0);
                                                standardDeviation.set(1, 0, 1.0);
                                                standardDeviation.set(2, 0, 2.0);
                                                Pose3d robotPose = leftBackMultiTagResult.get().estimatedPose;
                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                leftBackResult.getTimestampSeconds(),
                                                                standardDeviation);
                                        }
                                }

                                var leftFrontResult = peripherals.getLeftFrontCamResult();
                                Optional<EstimatedRobotPose> leftFrontMultiTagResult = leftFrontPhotonPoseEstimator
                                                .update(leftFrontResult);
                                if (leftFrontMultiTagResult.isPresent()) {
                                        if (leftFrontResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                                                standardDeviation.set(0, 0, 0.7);
                                                standardDeviation.set(1, 0, 0.7);
                                                standardDeviation.set(2, 0, 2.5);
                                                Pose3d robotPose = leftFrontMultiTagResult.get().estimatedPose;
                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                leftFrontResult.getTimestampSeconds(),
                                                                standardDeviation);
                                        }
                                }

                                double limelightAngVelRelToField = Constants.Vision.getLimelightAngVelRelToField(
                                                Globals.turretVelocity,
                                                getChassisSpeeds().omegaRadiansPerSecond);
                                if (Math.abs(limelightAngVelRelToField) < 2.0) {
                                        try {
                                                LimelightHelpers.SetRobotOrientation(Constants.Vision.LIMELIGHT_NAME,
                                                                gyro.getYawDegrees(),
                                                                limelightAngVelRelToField,
                                                                gyro.getPitchDegrees(), 0, -gyro.getRollDegrees(), 0);
                                                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                                                .getBotPoseEstimate_wpiBlue_MegaTag2(
                                                                                Constants.Vision.LIMELIGHT_NAME);

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
                                                        standardDeviation.set(0, 0, 2.0);
                                                        standardDeviation.set(1, 0, 2.0);
                                                        standardDeviation.set(2, 0, 5.0);
                                                        mt2Odometry.addVisionMeasurement(
                                                                        mt2.pose,
                                                                        mt2.timestampSeconds,
                                                                        standardDeviation);
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
                double x = mt2Odometry.getEstimatedPosition().getX();
                double y = mt2Odometry.getEstimatedPosition().getY();
                Rotation2d heading = gyro.getYaw();
                return new Pose2d(x, y, heading);
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
                Logger.recordOutput("Drive/RobotVelocities/X", avg.vxMetersPerSecond);
                Logger.recordOutput("Drive/RobotVelocities/Y", avg.vyMetersPerSecond);
                Logger.recordOutput("Drive/RobotVelocities/Omega", avg.omegaRadiansPerSecond);
                return avg;
        }

        @Override
        void update(DriveState currentState) {
                updateOdometryFusedArray(currentState);
                getChassisSpeeds();
                Logger.recordOutput("Robot/turret velocity filtered", Globals.turretVelocity);
                Logger.recordOutput("Robot/limelight ang vel rel to turret",
                                Constants.Vision.getLimelightAngVelRelToField(Globals.turretVelocity,
                                                getChassisSpeeds().omegaRadiansPerSecond));
                Logger.recordOutput("Robot/chassis speeds ang vel", getChassisSpeeds().omegaRadiansPerSecond);
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