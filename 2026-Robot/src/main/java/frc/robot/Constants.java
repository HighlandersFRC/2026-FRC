// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
                public static final String[] paths = new String[] {
                                "2AlgaeCenter.polarauto",
                                "2+1PieceFeeder.polarauto",
                                "3PieceFeederSmart.polarauto",
                                "4PieceFeederGroundSmart.polarauto",
                                "TushPush.polarauto",
                };

                public static int getSelectedPathIndex() {
                        if (OI.autoChooserConnected()) {
                                if (OI.autoChooser.getRawButton(1)) {
                                        return 0;
                                }
                                if (OI.autoChooser.getRawButton(2)) {
                                        return 1;
                                }
                                if (OI.autoChooser.getRawButton(3)) {
                                        return 2;
                                }
                                if (OI.autoChooser.getRawButton(4)) {
                                        return 3;
                                }
                                if (OI.autoChooser.getRawButton(5)) {
                                        return 4;
                                }
                        } else {
                                return (int) Math.round(SmartDashboard.getNumber("ROBOT AUTO OVERIDE", -1));
                        }
                        return -1;
                }

        }

        public static final double closedLoopSimResolution = 0.01; // seconds

        public static final double G = 9.80665;

        // Physical constants (e.g. field and robot dimensions)
        public static final class Physical {
                public static final double FIELD_WIDTH = inchesToMeters(316.64);
                public static final double FIELD_LENGTH = inchesToMeters(650.12);
                public static final double WHEEL_DIAMETER = inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
                public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;
                public static final double WHEEL_TO_FRAME_DISTANCE = inchesToMeters(2.5); // TODO: is this different for
                                                                                          // mk5s?
                public static final double TOP_SPEED = feetToMeters(30.0);
                public static final double SIM_TOP_SPEED = 6.0; // meters per second
                public static final double MAX_ACCELERATION = feetToMeters(30.0); // TODO: actually tune the top speed
                                                                                  // and max acceleration. Add a max
                                                                                  // deceleration if needed.
                public static final double SIM_MAX_ACCELERATION = 4.0; // meters per second
                public static final double ROBOT_LENGTH = inchesToMeters(26);
                public static final double ROBOT_WIDTH = inchesToMeters(26);
                public static final double MODULE_OFFSET = inchesToMeters(2.625); // TODO: is this different for mk5s?
                public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH / 2 - WHEEL_TO_FRAME_DISTANCE,
                                ROBOT_WIDTH / 2 - WHEEL_TO_FRAME_DISTANCE);
                public static final double SIM_MAX_ANGULAR_ACCELERATION = SIM_MAX_ACCELERATION / ROBOT_RADIUS;

                public static final double GRAVITY_ACCEL_MS2 = 9.806;

                public static final class Intake {
                        public static final int NUM_PIVOT_MOTORS = 1;
                        public static final int NUM_ROLLER_MOTORS = 1;
                        public static final double INTAKE_MASS_LB = 5.98; // TODO: make this the correct number
                        public static final double R_CG_M = 11.68; // TODO: make this the correct number
                        public static final double MOI = Units.lbsToKilograms(
                                        INTAKE_MASS_LB)
                                        * Math.pow(Units.inchesToMeters(R_CG_M), 2.0);
                }

                public static class Shooter {
                        public static final double SHOOTER_HEIGHT = 0.635;
                        public static final double TURRET_MAX_ROTATION_RADIANS = degreesToRadians(540);
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

                        public static double getTrajectoryHeight(double distanceFromHub) {
                                return 3 + 0.2 * distanceFromHub;
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
                }
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
                                return new Rotation2d(angleRadians);
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
                                double wheelRadiusMeters = Physical.Shooter.SHOOTER_WHEEL_RADIUS;
                                double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;
                                double wheelRotationsPerSecond = v / wheelCircumferenceMeters;
                                double wheelRPM = wheelRotationsPerSecond * 60;
                                return wheelRPM;
                        }
                }

                public static final class Intake {
                        public static final double INTAKE_DOWN_POSITION = Constants.degreesToRotations(90.0);
                        public static final double INTAKE_UP_POSITION = Constants.degreesToRotations(0.0);
                }

                public static final class Feeder {
                        public static final double HOPPER_PERCENT = 1.0 / 3.0;
                        public static final double LINEARIZER_PERCENT = 1.0 / 3.0;
                        public static final double LINEARIZER_SPEED_MPS = 1.0;
                        public static final double HOPPER_SPEED_MPS = 1.0;
                }
        }

        // PID constants
        public static final class PIDConstants {
                public static final class Turret {
                        public static final double kP0 = 100.0;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 5.0;
                }

                public static final class Hood {
                        public static final double kP0 = 8.0;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 0.0005;
                }

                public static final class Intake {
                        public static final double kP0 = 15.0; // TODO: make all of these actually good
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.5;
                        public static final double kG0 = 0.0;
                }
        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {

                // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
                // radians
                public static final double[] FRONT_CAMERA_POSE = { Constants.inchesToMeters(1.75),
                                Constants.inchesToMeters(11.625),
                                Constants.inchesToMeters(33.5), 0, -33.5, 0 };

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
                        // public static final double DRIVE_GEAR_RATIO = 7.03; // mk5 R1
                        public static final double DRIVE_GEAR_RATIO = 6.03; // mk5 R2
                        // public static final double DRIVE_GEAR_RATIO = 5.27; // mk5 R3
                        public static final double STEER_GEAR_RATIO = 26.09; // mk5
                }

                public static final class Shooter {
                        public static final double FLYWHEEL_GEAR_RATIO = 1 / 1.0;
                        public static final double HOOD_GEAR_RATIO = 180.73;
                        public static final double TURRET_GEAR_RATIO = 46.67;
                }

                public static final class Intake {
                        public static final double INTAKE_PIVOT_GEAR_RATIO = 1.0;
                }
        }

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
                public static final int TURRET_CANCODER_ONE_ID = 5;
                public static final int TURRET_CANCODER_TWO_ID = 6;

                // Intake
                public static final int INTAKE_PIVOT_MOTOR_ID = 13;
                public static final int INTAKE_ROLLER_MOTOR_ID = 14;

                // Feeder
                public static final int HOPPER_MOTOR_ID = 15;
                public static final int LINEARIZER_MOTOR_ID = 16;
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
