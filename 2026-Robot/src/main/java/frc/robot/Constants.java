// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.tools.math.Vector;

public final class Constants {
        public static final class Autonomous {
                public static final int STAGNATE_BOOST = 25;
                public static final int STAGNATE_THRESHOLD = 8; // Number of cycles of stagnation before ending path
                // lookahead distance is a function:
                // LOOKAHEAD = AUTONOMOUS_LOOKAHEAD_DISTANCE * velocity + MIN_LOOKAHEAD_DISTANCE
                // their constants
                public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 0.04; // Lookahead at 1m/s scaled by wanted
                                                                                 // velocity
                public static final double FULL_SEND_LOOKAHEAD = 0.60;
                public static final double MIN_LOOKAHEAD_DISTANCE = 0.05; // Lookahead distance at 0m/s
                // Path follower will end if within this radius of the final point
                public static final double AUTONOMOUS_END_ACCURACY = 0.40;
                public static final double ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY = 0.05;
                // When calculating the point distance, will divide x and y by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1.0;
                // When calculating the point distance, will divide theta by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = 4 * Math.PI;
                // Feed Forward Multiplier
                public static final double FEED_FORWARD_MULTIPLIER = 0.8044;
                public static final double ACCURATE_FOLLOWER_FEED_FORWARD_MULTIPLIER = 1;
                public static String[] paths;

                static {
                        ArrayList<String> autoPaths = new ArrayList<>();
                        File[] dir = Filesystem.getDeployDirectory().listFiles();
                        for (File file : dir) {
                                if (file.getName().contains(".polarauto")) {
                                        autoPaths.add(file.getName());
                                }
                        }
                        paths = new String[autoPaths.size()];
                        for (int i = 0; i < autoPaths.size(); i++) {
                                paths[i] = autoPaths.get(i);
                        }
                }

                public static int getSelectedPathIndex() {
                        String path = OI.getSelectedPath();
                        if (path.equals("None")) {
                                return -1;
                        }
                        for (int i = 0; i < paths.length; i++) {
                                if (path.equals(paths[i])) {
                                        return i;
                                }
                        }
                        return -1;
                }
        }

        public static final double G = 9.80665;

        // Physical constants (e.g. field and robot dimensions)
        public static final class Physical {
                public static final double FIELD_WIDTH = inchesToMeters(316.64);
                public static final double FIELD_LENGTH = inchesToMeters(650.12);
                public static final double WHEEL_DIAMETER = inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
                public static final double WHEEL_ROTATION_PER_METER = 1.0 / WHEEL_CIRCUMFERENCE;
                public static final double TOP_SPEED = feetToMeters(30.0);
                public static final double MAX_ACCELERATION = feetToMeters(30.0); // TODO: actually tune the top speed
                                                                                  // and max acceleration. Add a max
                                                                                  // deceleration if needed.
                public static final double ROBOT_LENGTH = inchesToMeters(24.5);
                public static final double ROBOT_WIDTH = inchesToMeters(29.5);
                public static final double MODULE_OFFSET = inchesToMeters(2.625); // Wheel to frame distance TODO: is
                                                                                  // this different for mk5s?
                public static final double ROBOT_RADIUS = Math.hypot((ROBOT_LENGTH / 2.0) - MODULE_OFFSET,
                                (ROBOT_WIDTH / 2.0) - MODULE_OFFSET);

                public static final double GRAVITY_ACCEL_MS2 = 9.806;

                public static final class Intake {
                        public static final int NUM_INTAKE_MOTORS = 1;
                        public static final int NUM_ROLLER_MOTORS = 1;
                        public static final double INTAKE_MASS_LB = 5.98; // TODO: make this the correct number
                        public static final double R_CG_M = 11.68; // TODO: make this the correct number
                        public static final double MOI = Units.lbsToKilograms(
                                        INTAKE_MASS_LB)
                                        * Math.pow(Units.inchesToMeters(R_CG_M), 2.0);
                }

                public static class Shooter {
                        public static final double SHOOTER_HEIGHT = 0.635;
                        public static final double TURRET_MAX_ROTATION_RADIANS = degreesToRadians(200);
                        public static final double SHOOTER_FLYWHEEL_ACCELERATION_RAD_S = Units
                                        .rotationsToRadians(4167 / 60);
                        public static final double SHOOTER_MAX_SPEED_RAD_S = Units.rotationsToRadians(10000 / 60);
                        public static final double SHOOTER_FRICTION_COEFFICIENT = SHOOTER_FLYWHEEL_ACCELERATION_RAD_S /
                                        SHOOTER_MAX_SPEED_RAD_S;
                        public static final double SHOOTER_WHEEL_RADIUS = inchesToMeters(2);
                        public static final int TURRET_MOTOR_COUNT = 1;
                        public static final double TURRET_MOI = 0.06; // kg*m^2
                        public static final int HOOD_MOTOR_COUNT = 1;
                        public static final double HOOD_MOI = 1 / 1684800; // kg*m^2
                        public static final Translation3d SHOOTER_POSITION = new Translation3d(
                                        inchesToMeters(20.0), inchesToMeters(20.0), SHOOTER_HEIGHT);
                        public static final double HOOD_ACCELERATION_RAD_S = degreesToRadians(100);
                        public static final double HOOD_MAX_SPEED_RAD_S = degreesToRadians(30);
                        public static final double HOOD_FRICTION_COEFFICIENT = HOOD_ACCELERATION_RAD_S /
                                        HOOD_MAX_SPEED_RAD_S;
                        public static final int TURRET_PULLEY_1_TOOTH_COUNT = 15;
                        public static final int TURRET_PULLEY_0_TOOTH_COUNT = 130;
                        public static final int TURRET_GEAR_2_TOOTH_COUNT = 39;
                        public static final int TURRET_GEAR_1_TOOTH_COUNT = 40;

                        public static double getTrajectoryHeight(double distanceFromHub) {
                                return 3 + 0.0 * distanceFromHub;
                        }
                }

                public static class Feeder {
                        public static final double LINEARIZER_MAX_SPEED_MPS = 3.0;
                        public static final double HOPPER_MAX_SPEED_MPS = 3.0;
                        public static final double HOPPER_ACCELERATION_MPS2 = 6.0;
                        public static final double LINEARIZER_ACCELERATION_MPS2 = 6.0;
                        public static final double HOPPER_FRICTION_COEFFICIENT = HOPPER_ACCELERATION_MPS2 /
                                        HOPPER_MAX_SPEED_MPS;
                        public static final double LINEARIZER_FRICTION_COEFFICIENT = LINEARIZER_ACCELERATION_MPS2 /
                                        LINEARIZER_MAX_SPEED_MPS;
                        public static final double LINEARIZER_SENSOR_TRIGGER_DISTANCE_M = inchesToMeters(3);
                        public static final double LINEARIZER_WHEEL_DIAMETER_M = inchesToMeters(3);
                        public static final double HOPPER_WHEEL_DIAMETER_M = inchesToMeters(1.25);
                }
        }

        public static final class Simulation {
                public static final double SIM_TOP_SPEED = 6.741; // meters per second
                public static final double SIM_STATIC_VELOCITY_THRESHOLD = 2.0; // meters per second
                public static final double SIM_BRAKE_MODE_THRESHOLD = 0.05;
                public static final double SIM_MAX_ACCELERATION = 15.0; // meters per second
                public static final double SIM_FRICTION_COEFFICIENT = SIM_MAX_ACCELERATION
                                / (SIM_TOP_SPEED * SIM_TOP_SPEED) * 0.4167;
                public static final double SIM_BRAKE_FRICTION_COEFFICIENT = 5.0 * SIM_FRICTION_COEFFICIENT;
                public static final double SIM_MAX_ANGULAR_ACCELERATION = SIM_MAX_ACCELERATION
                                / Constants.Physical.ROBOT_RADIUS;

                public static ChassisSpeeds getExpectedDriveSpeeds(double simTime, ChassisSpeeds current,
                                ChassisSpeeds wanted) {
                        Vector velocityVector = chassisSpeedsToVector(current);
                        Vector wantedVelocityVector = chassisSpeedsToVector(wanted);
                        double angularVelocity = current.omegaRadiansPerSecond;
                        double wantedAngularVelocity = wanted.omegaRadiansPerSecond;
                        int numSteps = (int) Math.floor(simTime / closedLoopSimResolution);
                        double dt = simTime / numSteps;
                        for (int i = 0; i < numSteps; i++) {
                                Vector acceleration = wantedVelocityVector.subtract(velocityVector);
                                if (acceleration.magnitude() > SIM_MAX_ACCELERATION) {
                                        acceleration = acceleration.scaled(SIM_MAX_ACCELERATION
                                                        / acceleration.magnitude());
                                }
                                Vector friction;
                                if (wantedVelocityVector.magnitude() < SIM_BRAKE_MODE_THRESHOLD) {
                                        friction = velocityVector.unit()
                                                        .scaled(SIM_STATIC_VELOCITY_THRESHOLD)
                                                        .sameDirectionSquare()
                                                        .scaled(-SIM_BRAKE_FRICTION_COEFFICIENT);
                                }
                                // else if (velocityVector
                                // .magnitude() < SIM_STATIC_VELOCITY_THRESHOLD) {
                                // friction = velocityVector.unit()
                                // .scaled(SIM_STATIC_VELOCITY_THRESHOLD)
                                // .sameDirectionSquare()
                                // .scaled(-SIM_FRICTION_COEFFICIENT);
                                // }
                                else {
                                        friction = velocityVector.sameDirectionSquare()
                                                        .scaled(-SIM_FRICTION_COEFFICIENT);
                                }
                                velocityVector = velocityVector.add(acceleration.scaled(dt)).add(friction.scaled(dt));
                                if (velocityVector.magnitude() > SIM_TOP_SPEED) {
                                        velocityVector = velocityVector.scaled(
                                                        SIM_TOP_SPEED / velocityVector.magnitude());
                                }
                                double angularAcceleration = Math.signum(wantedAngularVelocity - angularVelocity)
                                                * SIM_MAX_ANGULAR_ACCELERATION;
                                angularVelocity += angularAcceleration * dt;
                        }
                        return new ChassisSpeeds(velocityVector.getI(), velocityVector.getJ(), angularVelocity);
                }

                public static final double closedLoopSimResolution = RobotBase.isReal() ? 0.1 : 0.01; // seconds
        }

        public static final class Field {
                public static final double BLUE_HUB_X = inchesToMeters(182.1);
                public static final double RED_HUB_X = Constants.Physical.FIELD_LENGTH - BLUE_HUB_X;
                public static final double HUB_Y = Constants.Physical.FIELD_WIDTH / 2;
                public static final double HUB_Z = 1.83;
                public static final Translation3d HUB_POSE_BLUE = new Translation3d(BLUE_HUB_X, HUB_Y, HUB_Z);
                public static final Translation3d HUB_POSE_RED = new Translation3d(RED_HUB_X, HUB_Y, HUB_Z);
        }

        // Subsystem setpoint constants
        public static final class SetPoints {
                public static class Hood {
                        public static final double HOOD_MIN_ANGLE_RADIANS = degreesToRadians(55);
                        public static final double HOOD_MAX_ANGLE_RADIANS = degreesToRadians(85);
                        public static final double HOOD_PRECISION = degreesToRadians(1);

                        public static Rotation2d getHoodAngleSetpointForTrajectory(Translation3d trajectory) {
                                double dz = trajectory.getZ();
                                double dr = Math.hypot(trajectory.getX(), trajectory.getY());
                                double angleRadians = Math.atan(dz / dr);
                                return launchAngleToHoodAngle(new Rotation2d(angleRadians),
                                                shooterMPSToRPM(trajectory.getNorm()));
                        }

                        public static final Rotation2d hoodAngleToMotorAngle(Rotation2d hoodAngle) {
                                return Rotation2d.fromRadians(HOOD_MAX_ANGLE_RADIANS).minus(hoodAngle);
                        }

                        public static final Rotation2d motorAngleToHoodAngle(Rotation2d motorAngle) {
                                return Rotation2d.fromRadians(HOOD_MAX_ANGLE_RADIANS).minus(motorAngle);
                        }
                }

                public static class Turret {
                        public static final double TURRET_MIN_ANGLE_RADIANS = -Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
                        public static final double TURRET_MAX_ANGLE_RADIANS = Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
                        public static final double TURRET_PRECISION = degreesToRadians(5);

                        public static Rotation2d getTurretAngleSetpointForTrajectory(
                                        Translation3d _trajectorySetpoint) {
                                return new Rotation2d(Math.atan2(_trajectorySetpoint.getY(),
                                                _trajectorySetpoint.getX()));
                        }
                }

                public static class Flywheel {
                        public static final double FLYWHEEL_RPM_PRECISION = 100.0;

                        public static double getFlywheelRPMSetpointForTrajectory(Translation3d _trajectorySetpoint) {
                                double v = _trajectorySetpoint.getNorm();
                                double rpm = shooterMPSToRPM(v);
                                return rpm;
                        }
                }

                public static final class Intake {
                        public static final double INTAKE_DOWN_POSITION = Constants.degreesToRotations(90.0);
                        public static final double INTAKE_UP_POSITION = Constants.degreesToRotations(0.0);
                }

                public static final class Feeder {
                        public static final double HOPPER_PERCENT = 0.7;
                        public static final double LINEARIZER_PERCENT = 0.67;
                        public static final double LINEARIZER_SPEED_MPS = 1.0;
                        public static final double HOPPER_SPEED_MPS = 1.0;
                        public static final double LINEARIZER_AMPS = 60.0;
                        public static final double HOPPER_AMPS = 60.0;
                }
        }

        public static double shooterMPSToRPM(double mps) {
                return -369.004 * (1.17 - mps);
                // return 3621.1-11.9904*Math.sqrt(76609-8340*mps);
        }

        public static Rotation2d launchAngleToHoodAngle(Rotation2d launchAngle, double rpm) {
                return Rotation2d.fromDegrees(-1.26743 * (13.8 - launchAngle.getDegrees()));
        }

        // PID constants
        public static final class PIDConstants {
                public static final class Turret {
                        // Position PID
                        public static final double kP0 = 100.0;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 5.0;
                        // Motor Velocity PID
                        public static final double kP1 = 8.0;
                        public static final double kI1 = 0.0;
                        public static final double kD1 = 0.0;
                        public static final double kS1 = 0.0005;
                }

                public static final class Hood {
                        public static final double kP0 = 600.0;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.1;
                        public static final double kS0 = 0.0;
                        public static final double kG0 = 0.4;
                }

                public static final class Intake {
                        public static final double kP0 = 15.0; // TODO: make all of these actually good
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.5;
                        public static final double kG0 = 0.0;
                }

                public static final class Flywheel {
                        public static final double kP0 = 0.5;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.1;
                        public static final double kS0 = 0.0;
                        public static final double kV0 = 0.13;
                }
        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {

                public static final String LIMELIGHT_NAME = "limelight-goon";

                // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
                // radians
                public static final double[] FRONT_CAMERA_POSE = { Constants.inchesToMeters(1.75),
                                Constants.inchesToMeters(11.625),
                                Constants.inchesToMeters(33.5), 0, -33.5, 0 };

                public static final Translation3d LIMELIGHT_TO_TURRET_OFFSET = new Translation3d(
                                inchesToMeters(20.0), inchesToMeters(20.0), 0);

                public static final Rotation3d LIMELIGHT_ROTATION_RELATIVE_TO_TURRET = new Rotation3d(
                                0.0,
                                0.0,
                                0.0);
                // Standard deviation adjustments
                public static final double STANDARD_DEVIATION_SCALAR = 1;

                /**
                 * Calculates the standard deviation scalar based on the distance from the tag.
                 *
                 * @param dist The distance from the tag.
                 * @return The standard deviation scalar.
                 */
                public static double getTagDistStdDevScalar(double dist) {
                        return 0.0000520833 * Math.pow(dist, 4) + 0.000394571 * Math.pow(dist, 3)
                                        + 0.000440341 * Math.pow(dist, 2)
                                        + 0.0554117 * dist + 0.0298674;
                }

                /**
                 * Calculates the standard deviation scalar based on the number of detected
                 * tags.
                 *
                 * @param numTags The number of detected tags.
                 * @return The standard deviation scalar.
                 */
                public static double getNumTagStdDevScalar(int numTags) {
                        if (numTags == 0) {
                                return 99999;
                        } else if (numTags == 1) {
                                return 1;
                        } else if (numTags == 2) {
                                return 0.6;
                        } else {
                                return 0.75;
                        }
                }

                public static void updateLimelightPoseFromTurret(
                                Rotation2d turretYaw,
                                Translation3d turretOffsetFromRobot,
                                Translation3d cameraOffsetFromTurret,
                                Rotation3d cameraRotationRelativeToTurret,
                                String limelightName) {

                        Rotation3d turretRotation = new Rotation3d(0.0, 0.0, turretYaw.getRadians());
                        Translation3d cameraRelativeToRobot = turretOffsetFromRobot
                                        .plus(cameraOffsetFromTurret.rotateBy(turretRotation));
                        Rotation3d cameraRotationRelativeToRobot = turretRotation.plus(cameraRotationRelativeToTurret);
                        Pose3d limelightPose = new Pose3d(cameraRelativeToRobot, cameraRotationRelativeToRobot);

                        try {
                                LimelightHelpers.setCameraPose_RobotSpace(
                                                limelightName,
                                                limelightPose.getX(),
                                                limelightPose.getY(),
                                                limelightPose.getZ(),
                                                Math.toDegrees(limelightPose.getRotation().getX()),
                                                Math.toDegrees(limelightPose.getRotation().getY()),
                                                Math.toDegrees(limelightPose.getRotation().getZ()));
                        } catch (Exception e) {
                                System.out.println("Could not set limelight pose: " + e.getMessage());
                        }
                }

                /**
                 * Calculates the standard deviation of the x-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the x-coordinate.
                 */
                public static double getTagStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.010807566510145635)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation of the y-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the y-coordinate.
                 */
                public static double getTagStdDevY(double xOffset, double yOffset) {
                        return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the x-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the x-coordinate.
                 */
                public static double getTriStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.01955724864971872)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the y-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the y-coordinate.
                 */
                public static double getTriStdDevY(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.002615358015002413 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.008955462032388808)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                public static double distBetweenPose(Pose3d pose1, Pose3d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }

                public static double distBetweenPose2d(Pose2d pose1, Pose2d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }
        }

        // Gear ratios and conversions
        public static final class Ratios {
                public static final class Drive {
                        public static final double DRIVE_GEAR_RATIO = 7.03; // mk5 R1
                        // public static final double DRIVE_GEAR_RATIO = 6.03; // mk5 R2
                        // public static final double DRIVE_GEAR_RATIO = 5.27; // mk5 R3
                        public static final double STEER_GEAR_RATIO = 26.09; // mk5
                }

                public static final class Shooter {
                        public static final double FLYWHEEL_GEAR_RATIO = 1.0 / 1.0;
                        public static final double HOOD_GEAR_RATIO = 845.0 / 7.0;
                        public static final double TURRET_GEAR_RATIO = 455.0 / 9.0;
                }

                public static final class Intake {
                        public static final double INTAKE_PIVOT_GEAR_RATIO = 1.0;
                        public static final double INTAKE_ROLLER_GEAR_RATIO = 1.0;
                }

                public static final class Feeder {
                        public static final double HOPPER_GEAR_RATIO = 3.0 / 1.0;
                        public static final double LINEARIZER_GEAR_RATIO = 3.0 / 1.0;
                }
        }

        public static final ArrayList<String> paths = new ArrayList<String>();

        // Can info such as IDs
        public static final class CANInfo {
                public static final String CANBUS_NAME = "Canivore";

                // drive
                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
                public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 2;
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
                public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 4;
                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
                public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
                public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 8;
                public static final int FRONT_RIGHT_MODULE_CANCODER_ID = 1;
                public static final int FRONT_LEFT_MODULE_CANCODER_ID = 2;
                public static final int BACK_LEFT_MODULE_CANCODER_ID = 3;
                public static final int BACK_RIGHT_MODULE_CANCODER_ID = 4;

                // Lights
                public static final int CANDLE_ID_0 = 0;
                public static final int CANDLE_ID_1 = 1;
                public static final int CANDLE_ID_2 = 2;

                // Shooter
                public static final int FLYWHEEL_MASTER_ID = 9;
                public static final int FLYWHEEL_SLAVE_ID = 10;
                public static final int HOOD_MOTOR_ID = 11;
                public static final int TURRET_MOTOR_ID = 12;
                public static final int TURRET_CANCODER_ONE_ID = 5; // on driving pulley
                public static final int TURRET_CANCODER_TWO_ID = 6;

                // Intake
                public static final int INTAKE_PIVOT_MOTOR_ID = 13;
                public static final int INTAKE_ROLLER_MOTOR_ID = 14;

                // Feeder
                public static final int HOPPER_MOTOR_ID = 15;
                public static final int LINEARIZER_MOTOR_ID = 16;
                public static final int LINEARIZER_CANRANGE_ID = 0;
        }

        // Misc. controller values
        public static final class OperatorConstants {
                public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_STICK_DEADZONE = 0.03;
                public static final double RIGHT_STICK_DEADZONE = 0.05;
        }

        // Motor Specs (used for simulation)
        public static final class MotorSpecs {
                public static final class x44 {
                        public static final double X44_FREE_SPEED_RPM = 7530;
                        public static final double X44_STALL_TORQUE_NM = 4.05;
                        public static final double X44_STALL_CURRENT_A = 275;
                        public static final double X44_FREE_CURRENT_A = 1.4;
                        public static final double X44_NOMINAL_VOLTAGE_V = 12;

                        public static DCMotor getX44Gearbox(int numMotors) {
                                return new DCMotor(X44_NOMINAL_VOLTAGE_V, X44_STALL_TORQUE_NM, X44_STALL_CURRENT_A,
                                                X44_FREE_CURRENT_A, Units.rotationsPerMinuteToRadiansPerSecond(
                                                                X44_FREE_SPEED_RPM),
                                                numMotors);
                        }
                }
        }

        /**
         * Converts inches to meters.
         *
         * @param inches The length in inches to be converted.
         * @return The equivalent length in meters.
         */
        public static double inchesToMeters(double inches) {
                return inches / 39.37;
        }

        public static double metersToInches(double meters) {
                return meters * 39.37;
        }

        public static Vector chassisSpeedsToVector(ChassisSpeeds chassisSpeeds) {
                return new Vector(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        }

        /**
         * Converts feet to meters.
         *
         * @param inches The length in feet to be converted.
         * @return The equivalent length in meters.
         */
        public static double feetToMeters(double feet) {
                return feet / 3.281;
        }

        /**
         * Converts a quantity in rotations to radians.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double rotationsToRadians(double rotations) {
                return rotations * 2 * Math.PI;
        }

        /**
         * Converts a quantity in degrees to rotations.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in rotations.
         */
        public static double degreesToRotations(double degrees) {
                return degrees / 360;
        }

        /**
         * Converts a quantity in rotations to degrees.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in degrees.
         */
        public static double rotationsToDegrees(double rotations) {
                return rotations * 360;
        }

        /**
         * Converts a quantity in degrees to radians.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double degreesToRadians(double degrees) {
                return degrees * Math.PI / 180;
        }

        /**
         * Standardizes an angle to be within the range [0, 360) degrees.
         *
         * @param angleDegrees The input angle in degrees.
         * @return The standardized angle within the range [0, 360) degrees.
         */
        public static double standardizeAngleDegrees(double angleDegrees) {
                return ((angleDegrees % 360) + 360) % 360;
        }

        /**
         * Calculates the x-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The x-component of the unit vector.
         */
        public static double angleToUnitVectorI(double angle) {
                return (Math.cos(angle));
        }

        /**
         * Calculates the y-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The y-component of the unit vector.
         */
        public static double angleToUnitVectorJ(double angle) {
                return (Math.sin(angle));
        }

        /**
         * Converts revolutions per minute (RPM) to revolutions per second (RPS).
         *
         * @param RPM The value in revolutions per minute (RPM) to be converted.
         * @return The equivalent value in revolutions per second (RPS).
         */
        public static double RPMToRPS(double RPM) {
                return RPM / 60;
        }

        /**
         * Converts revolutions per second (RPS) to revolutions per minute (RPM).
         *
         * @param RPM The value in revolutions per second (RPS) to be converted.
         * @return The equivalent value in revolutions per minute (RPM).
         */
        public static double RPSToRPM(double RPS) {
                return RPS * 60;
        }

        /**
         * Standardizes an angle to be within the range [otherAngle - pi, otherAngle +
         * pi) radians.
         *
         * @param angle      The input angle in radians.
         * @param otherAngle The reference angle in radians.
         * @return The standardized angle within the range [otherAngle - pi, otherAngle
         *         +
         *         pi) radians.
         */
        public static double standardizeAngleToOther(double angle, double otherAngle) {
                double delta = angle - otherAngle;
                delta = Math.IEEEremainder(delta, 2 * Math.PI); // gives value in [-π, π]
                return otherAngle + delta;
        }

        /**
         * Standardizes an angle to be within the range [otherAngle - 180, otherAngle +
         * 180) degrees.
         *
         * @param angle      The input angle in degrees.
         * @param otherAngle The reference angle in degrees.
         * @return The standardized angle within the range [otherAngle - 180, otherAngle
         *         +
         *         180) degrees.
         */
        public static double standardizeAngleToOtherDegrees(double angle, double otherAngle) {
                return Math.toDegrees(standardizeAngleToOther(degreesToRadians(angle), degreesToRadians(otherAngle)));
        }
}
