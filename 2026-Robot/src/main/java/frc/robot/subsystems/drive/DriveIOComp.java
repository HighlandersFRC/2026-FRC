package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
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
        private Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
        private Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
        private Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
        private Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

        private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        private SwerveDrivePoseEstimator mt2Odometry;
        private Pose2d mt2Pose;
        private Peripherals peripherals;
        private int i = 0;
        private final Vector[] prevVelocities;
        private Pose2d prevPose = new Pose2d();

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
                prevVelocities = new Vector[10];
                for (int j = 0; j < prevVelocities.length; j++) {
                        prevVelocities[j] = new Vector();
                }
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
                try {
                        LimelightHelpers.SetRobotOrientation("limelight-goon",
                                        gyro.getYawDegrees(), 0, 0, 0, 0, 0);
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-goon");

                        // if our angular velocity is greater than 360 degrees per second, ignore vision
                        // updates
                        boolean doRejectUpdate = false;
                        // if (Math.abs(gyro.getAngularVelocityZDeviceDegPerSec()) > 360) {
                        // doRejectUpdate = true;
                        // }
                        if (mt2.tagCount == 0) {
                                doRejectUpdate = true;
                        }
                        if (!doRejectUpdate) {
                                // mt2Pose.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                                mt2Odometry.addVisionMeasurement(
                                                mt2.pose,
                                                mt2.timestampSeconds);
                        }
                } catch (Exception e) {
                        System.out.println(e);
                }
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
                Vector avg = new Vector();
                for (int j = 0; j < prevVelocities.length; j++) {
                        avg = avg.add(prevVelocities[j]);
                }
                avg = avg.scaled(1.0 / (prevVelocities.length));
                Logger.recordOutput("Robot Velocity/X", avg.getI());
                Logger.recordOutput("Robot Velocity/Y", avg.getJ());
                return avg;
        }

        private Vector getVelocityVectorNoDamp() {
                Pose2d current = mt2Odometry.getEstimatedPosition();
                Translation2d change = current.getTranslation().minus(prevPose.getTranslation());
                Translation2d velocity = change;
                if (Globals.loopPeriodSecs != 0) {
                        velocity = change.times(1 / Globals.loopPeriodSecs);
                }
                return new Vector(velocity.getX(), velocity.getY());
        }

        @Override
        void update(DriveState currentState) {
                updateOdometryFusedArray(currentState);
                prevVelocities[i] = getVelocityVectorNoDamp();
                prevPose = mt2Pose;
                i++;
                i = i % (prevVelocities.length - 1);
        }
}
