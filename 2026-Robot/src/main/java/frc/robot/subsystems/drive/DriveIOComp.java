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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
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

        PhotonPoseEstimator photonPoseEstimator;
        PhotonPoseEstimator backPhotonPoseEstimator;
        PhotonPoseEstimator backLeftPhotonPoseEstimator;
        PhotonPoseEstimator backRightPhotonPoseEstimator;

        // PhotonPoseEstimator rightPhotonPoseEstimator;
        // PhotonPoseEstimator leftPhotonPoseEstimator;
        PhotonPoseEstimator swervePhotonPoseEstimator;
        PhotonPoseEstimator gamePiecePhotonPoseEstimator;
        AprilTagFieldLayout aprilTagFieldLayout;

        // *********************NOTE THE PITCH IS POSITIVE DOWNWARDS
        // **********************************

        Transform3d frontReefRobotToCam = new Transform3d( // top front reef cam
                        new Translation3d(Constants.inchesToMeters(2.0), Constants.inchesToMeters(-11.5),
                                        Constants.inchesToMeters(23.625)),
                        new Rotation3d(Math.toRadians(0.3), Math.toRadians(25.6), Math.toRadians(15.0)));

        Transform3d frontSwerveRobotToCam = new Transform3d( // front reef cam on swerve module
                        new Translation3d(Constants.inchesToMeters(11.75),
                                        Constants.inchesToMeters(-8.5),
                                        Constants.inchesToMeters(8.75)),
                        new Rotation3d(Math.toRadians(1.1), Math.toRadians(15.4),
                                        Math.toRadians(35.0)));

        Transform3d backReefRobotToCam = new Transform3d( // top back reef cam
                        new Translation3d(Constants.inchesToMeters(-2.0), Constants.inchesToMeters(-11.5),
                                        Constants.inchesToMeters(23.625)),
                        new Rotation3d(Math.toRadians(2.8), Math.toRadians(25.9), Math.toRadians(165.0)));

        Transform3d backLeftReefRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(-12.375), Constants.inchesToMeters(9.375),
                                        Constants.inchesToMeters(8.6875)),
                        new Rotation3d(Math.toRadians(1.2), Math.toRadians(-19.7), Math.toRadians(181.53))); // 0.4,
                                                                                                             // -20.5

        Transform3d backRightReefRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(
                                        -12.375), Constants.inchesToMeters(-9.25),
                                        Constants.inchesToMeters(8.6875)),
                        new Rotation3d(Math.toRadians(1.5), Math.toRadians(-19.7), Math.toRadians(178.47)));

        Transform3d gamePieceReefRobotToCam = new Transform3d(
                        new Translation3d(Constants.inchesToMeters(2.0), Constants.inchesToMeters(-11.5),
                                        Constants.inchesToMeters(20.25)),
                        new Rotation3d(Math.toRadians(1.0), Math.toRadians(21.4), Math.toRadians(15.0)));

        // xy position of module based on robot width and distance from edge of robot
        private final double moduleX = ((Constants.Physical.ROBOT_LENGTH) / 2) - Constants.Physical.MODULE_OFFSET;
        private final double moduleY = ((Constants.Physical.ROBOT_WIDTH) / 2) - Constants.Physical.MODULE_OFFSET;

        // Locations for the swerve drive modules relative to the robot center.
        Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
        Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
        Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
        Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        SwerveDrivePoseEstimator mt2Odometry;
        Pose2d mt2Pose;
        Peripherals peripherals;

        public DriveIOComp(Peripherals peripherals) {
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
                                                        + "2025-reefscape-andymark.json");
                } catch (Exception e) {
                        java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
                }
                photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontReefRobotToCam);

                backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backReefRobotToCam);

                backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftReefRobotToCam);

                backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightReefRobotToCam);

                swervePhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontSwerveRobotToCam);

                // rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontBargeRobotToCam);

                // leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backBargeRobotToCam);

                gamePiecePhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, gamePieceReefRobotToCam);

                frontRight.init();
                frontLeft.init();
                backRight.init();
                backLeft.init();
        }

        @Override
        void zeroIMU() {
                gyro.setYaw(0.0);
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
        }

        @Override
        protected void setCurrentLimits(int supply, int stator) {
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
                mt2Pose = mt2Odometry.update(getYaw(), swerveModulePositions);

                Matrix<N3, N1> standardDeviation = new Matrix<>(Nat.N3(), Nat.N1());
                Logger.recordOutput("Closde to reef", closeToReef());

                if (((closeToReef()) || inReefInteractionState(
                                currentState))
                                && currentState != DriveState.REEF_MORE) {
                        photonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                        backPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                        backLeftPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                        backRightPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                        swervePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
                        gamePiecePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

                        Rotation2d robotRotation = getYaw();
                        double time = Timer.getFPGATimestamp();
                        photonPoseEstimator.addHeadingData(time, robotRotation);
                        backPhotonPoseEstimator.addHeadingData(time, robotRotation);
                        backLeftPhotonPoseEstimator.addHeadingData(time, robotRotation);
                        backRightPhotonPoseEstimator.addHeadingData(time, robotRotation);
                        swervePhotonPoseEstimator.addHeadingData(time, robotRotation);
                        gamePiecePhotonPoseEstimator.addHeadingData(time, robotRotation);
                } else {
                        photonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                        backPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                        backLeftPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                        backRightPhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                        swervePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                        gamePiecePhotonPoseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
                }
                Logger.recordOutput("Back Strategy: ",
                                backPhotonPoseEstimator.getPrimaryStrategy().toString());
                Logger.recordOutput("Back left strat: ", backLeftPhotonPoseEstimator.getPrimaryStrategy().toString());
                Logger.recordOutput("Back right strat: ",
                                backRightPhotonPoseEstimator.getPrimaryStrategy().toString());

                if (getRobotSpeed() < 2.4) {
                        var backResult = peripherals.getBackReefCamResult();
                        Optional<EstimatedRobotPose> backMultiTagResult = backPhotonPoseEstimator.update(backResult);
                        if (backMultiTagResult.isPresent()) {
                                if (backResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                && backResult.getBestTarget().fiducialId != 5
                                                && backResult.getBestTarget().fiducialId != 4
                                                && backResult.getBestTarget().fiducialId != 14
                                                && backResult.getBestTarget().fiducialId != 15
                                                && backResult.getBestTarget().fiducialId != 3
                                                && backResult.getBestTarget().fiducialId != 16) {
                                        Pose3d robotPose = backMultiTagResult.get().estimatedPose;
                                        Logger.recordOutput("multitag result", robotPose);
                                        int numFrontTracks = backResult.getTargets().size();
                                        Pose3d tagPose = aprilTagFieldLayout
                                                        .getTagPose(backResult.getBestTarget().getFiducialId()).get();
                                        double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                                        // Logger.recordOutput("Distance to tag", distToTag);
                                        if (distToTag < 3.2) {
                                                if (inReefInteractionState(currentState)) {
                                                        standardDeviation.set(0, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        if (backResult.getBestTarget()
                                                                        .getFiducialId() == getClosestTagId(
                                                                                        mt2Odometry.getEstimatedPosition())) {
                                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                                backResult.getTimestampSeconds());
                                                        }
                                                } else {
                                                        standardDeviation.set(0, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                        backResult.getTimestampSeconds());
                                                }
                                                // Pose2d poseWithoutAngle = new
                                                // Pose2d(robotPose.toPose2d().getTranslation(),
                                                // new Rotation2d(Math.toRadians(gyro.getYawDegrees())));
                                        }
                                }
                        }

                        var swerveResult = peripherals.getFrontSwerveCamResult();
                        Optional<EstimatedRobotPose> swerveMultiTagResult = swervePhotonPoseEstimator
                                        .update(swerveResult);
                        if (swerveMultiTagResult.isPresent()
                                        && (!inReefInteractionState(
                                                        currentState))) {
                                if (swerveResult.getBestTarget().getPoseAmbiguity() < 0.3) {
                                        Pose3d robotPose = swerveMultiTagResult.get().estimatedPose;
                                        int numFrontTracks = swerveResult.getTargets().size();
                                        Pose3d tagPose = aprilTagFieldLayout
                                                        .getTagPose(swerveResult.getBestTarget().getFiducialId()).get();
                                        double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                                        // Logger.recordOutput("Distance to tag", distToTag);
                                        if (distToTag < 3.2) {
                                                if (inReefInteractionState(currentState)) {
                                                        standardDeviation.set(0, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        if (swerveResult.getBestTarget()
                                                                        .getFiducialId() == getClosestTagId(
                                                                                        mt2Odometry.getEstimatedPosition())) {
                                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                                swerveResult.getTimestampSeconds());
                                                        }
                                                } else {
                                                        standardDeviation.set(0, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                        swerveResult.getTimestampSeconds());
                                                }
                                                // Pose2d poseWithoutAngle = new
                                                // Pose2d(robotPose.toPose2d().getTranslation(),
                                                // new Rotation2d(Math.toRadians(gyro.getYawDegrees())));
                                        }
                                }
                        }

                        var backLeftResult = peripherals.getBackLeftReefCamResult();
                        Optional<EstimatedRobotPose> backLeftMultiTagResult = backLeftPhotonPoseEstimator
                                        .update(backLeftResult);
                        if (backLeftMultiTagResult.isPresent()) {
                                if (backLeftResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                && backLeftResult.getBestTarget().fiducialId != 5
                                                && backLeftResult.getBestTarget().fiducialId != 4
                                                && backLeftResult.getBestTarget().fiducialId != 14
                                                && backLeftResult.getBestTarget().fiducialId != 15
                                                && backLeftResult.getBestTarget().fiducialId != 3
                                                && backLeftResult.getBestTarget().fiducialId != 16) {
                                        Pose3d robotPose = backLeftMultiTagResult.get().estimatedPose;
                                        int numFrontTracks = backLeftResult.getTargets().size();
                                        Pose3d tagPose = aprilTagFieldLayout
                                                        .getTagPose(backLeftResult.getBestTarget().getFiducialId())
                                                        .get();
                                        double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                                        if (distToTag < 3.2) {
                                                if (inReefInteractionState(currentState)) {
                                                        standardDeviation.set(0, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        if (backLeftResult.getBestTarget()
                                                                        .getFiducialId() == getClosestTagId(
                                                                                        mt2Odometry.getEstimatedPosition())) {
                                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                                backLeftResult.getTimestampSeconds());
                                                        }
                                                } else {
                                                        standardDeviation.set(0, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                        backLeftResult.getTimestampSeconds());
                                                }
                                                // Pose2d poseWithoutAngle = new
                                                // Pose2d(robotPose.toPose2d().getTranslation(),
                                                // new Rotation2d(Math.toRadians(gyro.getYawDegrees())));
                                        }
                                }
                        }

                        var backRightResult = peripherals.getBackRightReefCamResult();
                        Optional<EstimatedRobotPose> backRightMultiTagResult = backRightPhotonPoseEstimator
                                        .update(backRightResult);
                        if (backRightMultiTagResult.isPresent()) {
                                if (backRightResult.getBestTarget().getPoseAmbiguity() < 0.3
                                                && backRightResult.getBestTarget().fiducialId != 5
                                                && backRightResult.getBestTarget().fiducialId != 4
                                                && backRightResult.getBestTarget().fiducialId != 14
                                                && backRightResult.getBestTarget().fiducialId != 15
                                                && backRightResult.getBestTarget().fiducialId != 3
                                                && backRightResult.getBestTarget().fiducialId != 16) {
                                        Pose3d robotPose = backRightMultiTagResult.get().estimatedPose;
                                        int numFrontTracks = backRightResult.getTargets().size();
                                        Pose3d tagPose = aprilTagFieldLayout
                                                        .getTagPose(backRightResult.getBestTarget().getFiducialId())
                                                        .get();
                                        double distToTag = Constants.Vision.distBetweenPose(tagPose, robotPose);
                                        // Logger.recordOutput("Distance to tag", distToTag);
                                        if (distToTag < 3.2) {
                                                if (inReefInteractionState(currentState)) {
                                                        standardDeviation.set(0, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        0.5
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        if (backRightResult.getBestTarget()
                                                                        .getFiducialId() == getClosestTagId(
                                                                                        mt2Odometry.getEstimatedPosition())) {
                                                                mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                                backRightResult.getTimestampSeconds());
                                                        }
                                                } else {
                                                        standardDeviation.set(0, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(1, 0,
                                                                        Constants.Vision.getNumTagStdDevScalar(
                                                                                        numFrontTracks)
                                                                                        * Constants.Vision
                                                                                                        .getTagDistStdDevScalar(
                                                                                                                        distToTag));
                                                        // + Math.pow(dif,
                                                        // Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE)
                                                        // * Constants.Vision.ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR);
                                                        standardDeviation.set(2, 0, 0.9);

                                                        mt2Odometry.addVisionMeasurement(robotPose.toPose2d(),
                                                                        backRightResult.getTimestampSeconds());
                                                }
                                                // Pose2d poseWithoutAngle = new
                                                // Pose2d(robotPose.toPose2d().getTranslation(),
                                                // new Rotation2d(Math.toRadians(gyro.getYawDegrees())));
                                        }
                                }
                        }
                }
        }

        private int getClosestTagId(Pose2d pose) {
                int closestTag = 0;
                double closestDistance = Double.MAX_VALUE;

                for (int i = 1; i <= aprilTagFieldLayout.getTags().size(); i++) {
                        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(i);
                        if (tagPose.isPresent()) {
                                double distance = Constants.Vision.distBetweenPose2d(pose, tagPose.get().toPose2d());
                                if (distance < closestDistance) {
                                        closestDistance = distance;
                                        closestTag = i;
                                }
                        }
                }
                return closestTag;
        }

        private double getRobotSpeed() {
                return (Math.abs(frontLeft.getGroundSpeed()) + Math.abs(frontRight.getGroundSpeed())
                                + Math.abs(backLeft.getGroundSpeed())
                                + Math.abs(backRight.getGroundSpeed())) / 4.0;
        }

        private boolean inReefInteractionState(DriveState systemState) {
                return systemState == DriveState.L4_REEF || systemState == DriveState.L3_REEF ||
                                systemState == DriveState.REEF || systemState == DriveState.AUTO_L1 ||
                                systemState == DriveState.AUTO_L1_MORE || systemState == DriveState.ALGAE ||
                                systemState == DriveState.ALGAE_MORE || systemState == DriveState.ALGAE_MORE_MORE;
        }

        private boolean closeToReef() {
                double dist = DriverStation.isAutonomousEnabled() ? 2.7 : 2.3;
                if (distanceFromCenterOfReef() < dist) {
                        return true;
                } else {
                        return false;
                }
        }

        private double distanceFromCenterOfReef() {
                if (isOnBlueSide()) {
                        return Math.hypot(
                                        (mt2Odometry.getEstimatedPosition().getX() - Constants.Reef.centerBlue.getX()),
                                        ((mt2Odometry.getEstimatedPosition().getY()
                                                        - Constants.Reef.centerBlue.getY())));
                } else {
                        return Math.hypot((mt2Odometry.getEstimatedPosition().getX() - Constants.Reef.centerRed.getX()),
                                        ((mt2Odometry.getEstimatedPosition().getY()
                                                        - Constants.Reef.centerRed.getY())));
                }
        }

        private boolean isOnBlueSide() {
                return mt2Odometry.getEstimatedPosition().getX() < Constants.Physical.FIELD_LENGTH / 2.0;
        }

        @Override
        protected Pose2d getPosition() {
                return mt2Odometry.getEstimatedPosition();
        }

        @Override
        protected void drive(Vector velocityVector, double turnVelocity) {
                double yaw = getYaw().getRadians();
                frontLeft.drive(velocityVector, turnVelocity, yaw);
                frontRight.drive(velocityVector, turnVelocity, yaw);
                backLeft.drive(velocityVector, turnVelocity, yaw);
                backRight.drive(velocityVector, turnVelocity, yaw);
        }

        @Override
        protected void driveRobotCentric(Vector velocityVector, double turnRadiansPerSec) {
                frontLeft.drive(velocityVector, turnRadiansPerSec, 0);
                frontRight.drive(velocityVector, turnRadiansPerSec, 0);
                backLeft.drive(velocityVector, turnRadiansPerSec, 0);
                backRight.drive(velocityVector, turnRadiansPerSec, 0);
        }

        @Override
        protected void driveCamCentric(Vector velocityVector, double turnRadiansPerSec, double camAngle) {
                frontLeft.drive(velocityVector, turnRadiansPerSec, camAngle);
                frontRight.drive(velocityVector, turnRadiansPerSec, camAngle);
                backLeft.drive(velocityVector, turnRadiansPerSec, camAngle);
                backRight.drive(velocityVector, turnRadiansPerSec, camAngle);
        }

        @Override
        protected Vector getVelocityVector() {
                Vector velocityVector = new Vector();
                double pigeonAngleRadians = getYaw().getRadians();

                double frV = this.frontRight.getGroundSpeed();
                double frTheta = this.frontRight.getWheelPosition() + pigeonAngleRadians;
                double frVX = frV * Math.cos(frTheta);
                double frVY = frV * Math.sin(frTheta);
                double flV = this.frontLeft.getGroundSpeed();
                double flTheta = this.frontLeft.getWheelPosition() + pigeonAngleRadians;
                double flVX = flV * Math.cos(flTheta);
                double flVY = flV * Math.sin(flTheta);
                double blV = this.backLeft.getGroundSpeed();
                double blTheta = this.backLeft.getWheelPosition() + pigeonAngleRadians;
                double blVX = blV * Math.cos(blTheta);
                double blVY = blV * Math.sin(blTheta);
                double brV = this.backRight.getGroundSpeed();
                double brTheta = this.backRight.getWheelPosition() + pigeonAngleRadians;
                double brVX = brV * Math.cos(brTheta);
                double brVY = brV * Math.sin(brTheta);

                velocityVector.setI(frVX + flVX + blVX + brVX);
                velocityVector.setJ(frVY + flVY + blVY + brVY);
                return velocityVector;
        }

        @Override
        void update(DriveState currentState) {
                updateOdometryFusedArray(currentState);
        }
}
