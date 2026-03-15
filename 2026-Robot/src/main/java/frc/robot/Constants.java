// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.tools.logging.TunableNumber;
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
                public static final double FULL_SEND_LOOKAHEAD = 0.6741;
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
                public static final double SLOW_FOLLOWER_MULTIPLIER = 0.95;
                public static final double FULL_SEND_FOLLOWER_MULTIPLIER = 1.2;
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
                public static final double DRIVE_ACCELERATION_WHEN_SHOOTING = 0.67; // percentage of the wanted
                                                                                    // acceleration when shooting [0, 1]
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

                // 1.437/1.736 4.115
                public static Pose2d climbPoseLeftBlueSide = new Pose2d(new Translation2d(
                                1.398,
                                4.272),
                                new Rotation2d(Math.toRadians(-90.0)));
                public static Pose2d preClimbPoseLeftBlueSide = new Pose2d(new Translation2d(
                                1.736,
                                4.272),
                                new Rotation2d(Math.toRadians(-90.0)));
                public static Pose2d climbPoseRightBlueSide = new Pose2d(new Translation2d(
                                climbPoseLeftBlueSide.getTranslation().getX(),
                                climbPoseLeftBlueSide.getTranslation().getY() - inchesToMeters(33.75)),
                                new Rotation2d(Math.toRadians(-90.0)));
                public static Pose2d preClimbPoseRightBlueSide = new Pose2d(new Translation2d(
                                preClimbPoseLeftBlueSide.getTranslation().getX(),
                                preClimbPoseLeftBlueSide.getTranslation().getY() - inchesToMeters(33.75)),
                                new Rotation2d(Math.toRadians(-90.0)));

                // public static Pose2d climbPoseLeftRedSide = new Pose2d(new Translation2d(
                // 14.94, 3.94), new Rotation2d(Math.PI / 2));
                // public static Pose2d preClimbPoseLeftRedSide = new Pose2d(new Translation2d(
                // 14.6, 3.94), new Rotation2d(Math.PI / 2));

                // public static Pose2d climbPoseRightRedSide = new Pose2d(new Translation2d(
                // 14.94, 4.84), new Rotation2d(Math.PI / 2));
                // public static Pose2d preClimbPoseRightRedSide = new Pose2d(new Translation2d(
                // 14.6, 4.84), new Rotation2d(Math.PI / 2));

                // 33.75 in
                public static Pose2d climbPoseLeftRedSide = new Pose2d(new Translation2d(
                                FIELD_LENGTH - climbPoseLeftBlueSide.getX(),
                                FIELD_WIDTH - climbPoseLeftBlueSide.getY()), new Rotation2d(Math.PI / 2));
                public static Pose2d preClimbPoseLeftRedSide = new Pose2d(new Translation2d(
                                FIELD_LENGTH - preClimbPoseLeftBlueSide.getX(),
                                FIELD_WIDTH - preClimbPoseLeftBlueSide.getY()), new Rotation2d(Math.PI / 2));

                public static Pose2d climbPoseRightRedSide = new Pose2d(new Translation2d(
                                FIELD_LENGTH - climbPoseRightBlueSide.getX(),
                                FIELD_WIDTH - climbPoseRightBlueSide.getY()), new Rotation2d(Math.PI / 2));
                public static Pose2d preClimbPoseRightRedSide = new Pose2d(new Translation2d(
                                FIELD_LENGTH - preClimbPoseRightBlueSide.getX(),
                                FIELD_WIDTH - preClimbPoseRightBlueSide.getY()), new Rotation2d(Math.PI / 2));

                public static final class Drive {
                        public static final double xAccelLimit = 3.5;
                        public static final double yAccelLimit = 3.5;
                        public static final double xDebounceLimit = 0.2;
                        public static final double yDebounceLimit = 0.2;
                        public static final double velLookaheadTime = 0.03;
                }

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
                                        inchesToMeters(0.0), inchesToMeters(1.75), inchesToMeters(21.44));
                        public static final double HOOD_ACCELERATION_RAD_S = degreesToRadians(100);
                        public static final double HOOD_MAX_SPEED_RAD_S = degreesToRadians(30);
                        public static final double HOOD_FRICTION_COEFFICIENT = HOOD_ACCELERATION_RAD_S /
                                        HOOD_MAX_SPEED_RAD_S;
                        public static final double TURRET_PULLEY_1_TOOTH_COUNT = 15;
                        public static final double TURRET_PULLEY_0_TOOTH_COUNT = 134;
                        public static final double TURRET_GEAR_2_TOOTH_COUNT = 31;
                        public static final double TURRET_GEAR_1_TOOTH_COUNT = 60;
                        public static final double SHOT_DEBOUNCE_S = 0.08;
                        public static final double SHOT_ACCEL_LOW = -30.0;
                        public static final double SHOT_ACCEL_HIGH = 36.0;
                        public static final double SHOT_RPM_DELTA_LOW = -180.0;
                        public static final double SHOT_RPM_DELTA_HIGH = 200.0;
                        public static final double SHOT_SPIKE_CURRENT = 27.0;

                        public static double getTrajectoryHeight(double distanceFromHub) {
                                return 4 + 0.0 * distanceFromHub;
                        }
                }

                public static class Feeder {
                        public static final double DYE_ROTOR_MAX_SPEED_MPS = 3.0;
                        public static final double DYE_ROTOR_ACCELERATION_MPS2 = 6.0;
                        public static final double DYE_ROTOR_FRICTION_COEFFICIENT = DYE_ROTOR_ACCELERATION_MPS2 /
                                        DYE_ROTOR_MAX_SPEED_MPS;
                        public static final double DYE_ROTOR_WHEEL_DIAMETER_M = inchesToMeters(15);
                }
        }

        public static final class Simulation {
                public static final double SIM_TOP_SPEED = 9.72; // meters per second
                public static final double SIM_STATIC_VELOCITY_THRESHOLD = 2.0; // meters per second
                public static final double SIM_BRAKE_MODE_THRESHOLD = 0.05;
                public static final double SIM_MAX_ACCELERATION = 12.0; // meters per second
                public static final double SIM_FRICTION_COEFFICIENT = SIM_MAX_ACCELERATION
                                / (SIM_TOP_SPEED * SIM_TOP_SPEED) * 0.4499;
                public static final double SIM_BRAKE_FRICTION_COEFFICIENT = 10.0 * SIM_FRICTION_COEFFICIENT;
                public static final double SIM_MAX_ANGULAR_ACCELERATION = SIM_MAX_ACCELERATION
                                / Constants.Physical.ROBOT_RADIUS;

                public static ChassisSpeeds getExpectedDriveSpeeds(double simTime, ChassisSpeeds current,
                                ChassisSpeeds wanted) {
                        Vector velocityVector = chassisSpeedsToVector(current);
                        Vector wantedVelocityVector = chassisSpeedsToVector(wanted);
                        wantedVelocityVector.setJ(wantedVelocityVector.getJ() * -1); // Invert Y axis for simulation
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
                public static final double BUMP_LENGTH = 1.5; // 1.12776 actual

                public static final double BLUE_HUB_X = inchesToMeters(182.1);
                public static final double RED_HUB_X = Constants.Physical.FIELD_LENGTH - BLUE_HUB_X;
                public static final double HUB_Y = Constants.Physical.FIELD_WIDTH / 2;
                public static final double HUB_Z = 1.83;
                public static final Translation3d HUB_POSE_BLUE = new Translation3d(BLUE_HUB_X, HUB_Y, HUB_Z);
                public static final Translation3d HUB_POSE_RED = new Translation3d(RED_HUB_X, HUB_Y, HUB_Z);
                public static final double BUMP_WIDTH = inchesToMeters(44.4);

                public static final Translation2d RED_LEFT_FEED_POSE = new Translation2d(
                                13.0, 2.35);
                public static final Translation2d RED_RIGHT_FEED_POSE = new Translation2d(RED_LEFT_FEED_POSE.getX(),
                                Constants.Physical.FIELD_WIDTH - RED_LEFT_FEED_POSE.getY());
                public static final Translation2d BLUE_LEFT_FEED_POSE = new Translation2d(
                                Constants.Physical.FIELD_LENGTH - RED_LEFT_FEED_POSE.getX(),
                                RED_RIGHT_FEED_POSE.getY());
                public static final Translation2d BLUE_RIGHT_FEED_POSE = new Translation2d(BLUE_LEFT_FEED_POSE.getX(),
                                RED_LEFT_FEED_POSE.getY());

                public static Translation3d getHubPose() {
                        if (Globals.fieldSide.equals("blue")) {
                                return HUB_POSE_BLUE;
                        } else {
                                return HUB_POSE_RED;
                        }
                }

                public static Translation2d getFeedTarget(Translation2d turretPose) {
                        if (Globals.fieldSide.equals("blue")) {
                                if (turretPose.getY() > Constants.Physical.FIELD_WIDTH / 2) {
                                        return BLUE_LEFT_FEED_POSE;
                                } else {
                                        return BLUE_RIGHT_FEED_POSE;
                                }
                        } else {
                                if (turretPose.getY() < Constants.Physical.FIELD_WIDTH / 2) {
                                        return RED_LEFT_FEED_POSE;
                                } else {
                                        return RED_RIGHT_FEED_POSE;
                                }
                        }
                }

                public static boolean isInAllianceZone(Translation2d robotPosition) {
                        if (Globals.fieldSide.equals("red")) {
                                return HUB_POSE_RED.getX() + BUMP_WIDTH / 2 < robotPosition.getX();
                        } else {
                                return robotPosition.getX() < HUB_POSE_BLUE.getX() - BUMP_WIDTH / 2;
                        }
                }

                public static boolean isOnBump(Translation2d robotPosition) {
                        boolean onBlueBump = Math.abs(robotPosition.getX() - HUB_POSE_BLUE.getX()) < BUMP_WIDTH / 2;
                        boolean onRedBump = Math.abs(robotPosition.getX() - HUB_POSE_RED.getX()) < BUMP_WIDTH / 2;
                        return onBlueBump || onRedBump;
                }

                public static final double HUB_RADIUS = inchesToMeters(21.0);
                public static final double FEED_RADIUS = inchesToMeters(33.39);
                public static final double BALL_WIDTH = 0.15;
        }

        public class DynamicPassing { // chatgpt ahh code for dynamic passing

                private static final double RED_TARGET_X = 13.0;

                private static final double RED_LEFT_MIN_TARGET_Y = 1.5;
                private static final double RED_LEFT_MAX_TARGET_Y = 3.4;
                private static final double RED_LEFT_BUMP_MIDPOINT_Y = 2.5;

                private static final double RED_RIGHT_MIN_TARGET_Y = Constants.Physical.FIELD_WIDTH
                                - RED_LEFT_MAX_TARGET_Y;
                private static final double RED_RIGHT_MAX_TARGET_Y = Constants.Physical.FIELD_WIDTH
                                - RED_LEFT_MIN_TARGET_Y;
                private static final double RED_RIGHT_BUMP_MIDPOINT_Y = Constants.Physical.FIELD_WIDTH
                                - RED_LEFT_BUMP_MIDPOINT_Y;

                private static final double BLUE_TARGET_X = Constants.Physical.FIELD_LENGTH - RED_TARGET_X;

                private static final double BLUE_LEFT_MIN_TARGET_Y = RED_RIGHT_MIN_TARGET_Y;
                private static final double BLUE_LEFT_MAX_TARGET_Y = RED_RIGHT_MAX_TARGET_Y;
                private static final double BLUE_LEFT_BUMP_MIDPOINT_Y = RED_RIGHT_BUMP_MIDPOINT_Y;

                private static final double BLUE_RIGHT_MIN_TARGET_Y = RED_LEFT_MIN_TARGET_Y;
                private static final double BLUE_RIGHT_MAX_TARGET_Y = RED_LEFT_MAX_TARGET_Y;
                private static final double BLUE_RIGHT_BUMP_MIDPOINT_Y = RED_LEFT_BUMP_MIDPOINT_Y;

                private static final double X_SCALE = 0.09;

                public static Translation2d getTarget(Pose2d robotPose) {
                        if (Globals.fieldSide.equals("red")) {
                                if (robotPose.getY() < Constants.Physical.FIELD_WIDTH / 2) {
                                        return getTargetRedLeft(robotPose);
                                } else {
                                        return getTargetRedRight(robotPose);
                                }
                        } else {
                                if (robotPose.getY() > Constants.Physical.FIELD_WIDTH / 2) {
                                        return getTargetBlueLeft(robotPose);
                                } else {
                                        return getTargetBlueRight(robotPose);
                                }
                        }
                }

                public static Translation2d getTargetRedLeft(Pose2d robotPose) {
                        double rx = robotPose.getX();
                        double ry = robotPose.getY();
                        double normalizedY = ry / (Constants.Physical.FIELD_WIDTH * 0.5);
                        double targetY = RED_LEFT_MAX_TARGET_Y
                                        - normalizedY * (RED_LEFT_MAX_TARGET_Y - RED_LEFT_MIN_TARGET_Y);
                        double xAdjustment = (rx - RED_TARGET_X) * X_SCALE;
                        double factor = (ry - RED_LEFT_BUMP_MIDPOINT_Y) / 3.0;
                        factor = Math.max(-1.0, Math.min(1.0, factor));
                        targetY -= xAdjustment * factor;
                        targetY = Math.max(RED_LEFT_MIN_TARGET_Y, Math.min(RED_LEFT_MAX_TARGET_Y, targetY));
                        return new Translation2d(RED_TARGET_X, targetY);
                }

                public static Translation2d getTargetRedRight(Pose2d robotPose) {
                        double rx = robotPose.getX();
                        double ry = robotPose.getY();
                        double normalizedY = (Constants.Physical.FIELD_WIDTH + ry)
                                        / (Constants.Physical.FIELD_WIDTH * 0.5);
                        double targetY = (RED_RIGHT_MAX_TARGET_Y
                                        - normalizedY * (RED_RIGHT_MAX_TARGET_Y - RED_RIGHT_MIN_TARGET_Y));
                        targetY = RED_RIGHT_BUMP_MIDPOINT_Y + targetY;
                        double xAdjustment = (rx - RED_TARGET_X) * X_SCALE;
                        double factor = (ry - RED_RIGHT_BUMP_MIDPOINT_Y) / 3.0;
                        factor = Math.max(-1.0, Math.min(1.0, factor));
                        targetY -= xAdjustment * factor;
                        targetY = Math.max(
                                        RED_RIGHT_MIN_TARGET_Y,
                                        Math.min(RED_RIGHT_MAX_TARGET_Y, targetY));
                        return new Translation2d(RED_TARGET_X, targetY);
                }

                public static Translation2d getTargetBlueLeft(Pose2d robotPose) {
                        double rx = robotPose.getX();
                        double ry = robotPose.getY();
                        double normalizedY = (Constants.Physical.FIELD_WIDTH + ry)
                                        / (Constants.Physical.FIELD_WIDTH * 0.5);
                        double targetY = (BLUE_LEFT_MAX_TARGET_Y
                                        - normalizedY * (BLUE_LEFT_MAX_TARGET_Y - BLUE_LEFT_MIN_TARGET_Y));
                        targetY = BLUE_LEFT_BUMP_MIDPOINT_Y + targetY;
                        double xAdjustment = (rx - BLUE_TARGET_X) * X_SCALE;
                        double factor = (ry - BLUE_LEFT_BUMP_MIDPOINT_Y) / 3.0;
                        factor = Math.max(-1.0, Math.min(1.0, factor));
                        targetY += xAdjustment * factor;
                        targetY = Math.max(
                                        BLUE_LEFT_MIN_TARGET_Y,
                                        Math.min(BLUE_LEFT_MAX_TARGET_Y, targetY));
                        return new Translation2d(BLUE_TARGET_X, targetY);
                }

                public static Translation2d getTargetBlueRight(Pose2d robotPose) {
                        double rx = robotPose.getX();
                        double ry = robotPose.getY();
                        double normalizedY = ry / (Constants.Physical.FIELD_WIDTH * 0.5);
                        double targetY = BLUE_RIGHT_MAX_TARGET_Y
                                        - normalizedY * (BLUE_RIGHT_MAX_TARGET_Y - BLUE_RIGHT_MIN_TARGET_Y);
                        double xAdjustment = (rx - BLUE_TARGET_X) * X_SCALE;
                        double factor = (ry - BLUE_RIGHT_BUMP_MIDPOINT_Y) / 3.0;
                        factor = Math.max(-1.0, Math.min(1.0, factor));
                        targetY += xAdjustment * factor;
                        targetY = Math.max(BLUE_RIGHT_MIN_TARGET_Y, Math.min(BLUE_RIGHT_MAX_TARGET_Y, targetY));
                        return new Translation2d(BLUE_TARGET_X, targetY);
                }
        }

        // Subsystem setpoint constants
        public static final class SetPoints {
                public static class Hood {
                        public static final double HOOD_MIN_ANGLE_RADIANS = degreesToRadians(55.0);
                        public static final double HOOD_MAX_ANGLE_RADIANS = degreesToRadians(85.0);
                        public static final double HOOD_PRECISION = degreesToRadians(2.0);
                        public static final double HOOD_FEED_PRECISION = degreesToRadians(5.0);

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
                        public static final double TURRET_MIN_ANGLE_RADIANS = -Math.toRadians(360.0);
                        public static final double TURRET_MAX_ANGLE_RADIANS = Math.toRadians(90.0);
                        public static final double TURRET_PRECISION = degreesToRadians(1.476);

                        public static Rotation2d getTurretAngleSetpointForTrajectory(
                                        Translation3d _trajectorySetpoint) {
                                return new Rotation2d(Math.atan2(_trajectorySetpoint.getY(),
                                                _trajectorySetpoint.getX()));
                        }

                        public static Rotation2d getFutureSetpointEstimate(Rotation2d currentSetpoint,
                                        double driveAngularVelocity, double foresightTime) {
                                // Logger.recordOutput("Shooter/Turret Drive Angular Velocity",
                                // driveAngularVelocity);
                                double predictedAngle = currentSetpoint.getRadians()
                                                - driveAngularVelocity * foresightTime;
                                return new Rotation2d(predictedAngle);
                        }
                }

                public static class Flywheel {
                        public static final double FLYWHEEL_RPM_PRECISION = 200.0;
                        public static final double FLYWHEEL_RPM_FEED_PRECISION = 400.0;

                        public static double getFlywheelRPMSetpointForTrajectory(Translation3d _trajectorySetpoint) {
                                double v = _trajectorySetpoint.getNorm();
                                double rpm = shooterMPSToRPM(v);
                                return rpm;
                        }
                }

                public static class Shooter {
                        private final static double DISTANCE_OFFSET = 0.0;
                        private final static double ANGLE_OFFSET = 0.0;
                        private final static double RPM_OFFSET = 0.0;
                        private final static double TOF_OFFSET = 0.0;
                        // Distance in meters, Hood Angle, Flywheel RPM, Time of Flight in seconds
                        public static final double[][] SHOT_MAP = { { 1.372, 85, 1850, 0.82 },
                                        { 1.627, 85, 2000, 0.97 },
                                        { 1.987, 82, 2000, 0.96 },
                                        { 2.285, 80, 2250, 1.06 },
                                        { 2.502, 78, 2300, 1.11 },
                                        { 2.755, 77, 2350, 1.14 },
                                        { 3.079, 76, 2400, 1.11 },
                                        { 3.26, 75, 2450, 1.13 },
                                        { 3.597, 74, 2500, 1.19 },
                                        { 3.76, 73, 2525, 1.11 },
                                        { 3.993, 72, 2575, 1.16 },
                                        { 4.242, 71, 2650, 1.19 },
                                        { 4.524, 70, 2750, 1.29 },
                                        { 4.878, 69, 2750, 1.19 },
                                        { 5.112, 69, 2850, 1.24 },
                                        { 5.304, 69, 2900, 1.30 },
                                        { 5.65, 69, 2980, 1.30 },
                                        { 5.997, 67, 3150, 1.33 },
                                        { 6.235, 65, 3250, 1.26 },
                                        { 6.507, 63, 3300, 1.21 },
                                        { 6.7, 59, 3450, 1.20 }
                        };

                        private final static double FEED_DISTANCE_OFFSET = 0.0;
                        private final static double FEED_ANGLE_OFFSET = 0.0;
                        private final static double FEED_RPM_OFFSET = 0.0;
                        private final static double FEED_TOF_OFFSET = 0.0;
                        // Distance in meters, Hood Angle, Flywheel RPM, Time of Flight in seconds
                        public static final double[][] FEED_SHOT_MAP = new double[][] {
                                        { 1, 60, 700, 0.74 },
                                        { 1.5, 60, 900, 0.85 },
                                        { 1.79, 60, 1254, 0.84 },
                                        { 2.5, 60, 1400, 1.02 },
                                        { 3.01, 60, 1600, 1.0 },
                                        { 3.2, 60, 1750, 1.0 },
                                        { 3.49, 60, 1800, 1.01 },
                                        { 4.04, 60, 1900, 1.14 },
                                        { 4.48, 60, 2000, 1.21 },
                                        { 5.01, 60, 2150, 1.15 },
                                        { 5.52, 60, 2250, 1.35 },
                                        { 5.99, 60, 2350, 1.44 },
                                        { 6.42, 60, 2550, 1.38 },
                                        { 6.99, 60, 2700, 1.52 },
                                        { 7.51, 60, 2850, 1.57 },
                        };

                        static {
                                for (int i = 0; i < SHOT_MAP.length; i++) {
                                        SHOT_MAP[i][0] += DISTANCE_OFFSET;
                                        SHOT_MAP[i][1] += ANGLE_OFFSET;
                                        SHOT_MAP[i][2] += RPM_OFFSET;
                                        SHOT_MAP[i][3] += TOF_OFFSET;
                                }

                                for (int i = 0; i < FEED_SHOT_MAP.length; i++) {
                                        FEED_SHOT_MAP[i][0] += FEED_DISTANCE_OFFSET;
                                        FEED_SHOT_MAP[i][1] += FEED_ANGLE_OFFSET;
                                        FEED_SHOT_MAP[i][2] += FEED_RPM_OFFSET;
                                        FEED_SHOT_MAP[i][3] += FEED_TOF_OFFSET;
                                }
                        }
                }

                public static final class Intake {
                        public static final double INTAKE_DOWN_POSITION = 0.521484 + 25.076172;
                        public static final double INTAKE_UP_POSITION = Constants.degreesToRotations(0.0);
                        public static final double INTAKE_SHOOT_POSITION = 10.0;
                }

                public static final class Feeder {
                        public static final double DYE_ROTOR_PERCENT = 0.7;
                        public static final double DYE_ROTOR_SPEED_MPS = 1.0;
                        public static final double DYE_ROTOR_AMPS = 60.0;
                }

                public static final class Climber { // TODO: tune ALL OF THESE because they are just guesses
                        public static final double CLIMBER_HOOKS_RETRACT_ZONE = 1.0; // climber position that the hooks
                                                                                     // come out at (assuming climber
                                                                                     // initializes to zero)
                        public static final double CLIMBER_L1_EXTEND_HEIGHT_INCHES = 20.25; // climber position to get
                                                                                            // ready to grab l1
                        public static final double CLIMBER_AUTON_L1_RETRACT_HEIGHT_INCHES = 15.0; // climber position to
                                                                                                  // hover off the
                                                                                                  // ground in auto
                                                                                                  // (already latched on
                                                                                                  // L1) // 15.0
                        public static final double CLIMBER_L2_EXTEND_HEIGHT_INCHES = 0.8; // climber position to get
                                                                                          // ready to grab L2 (already
                                                                                          // latched on L1) // 0.4
                        public static final double CLIMBER_L3_EXTEND_HEIGHT_INCHES = 19.85; // climber position to get
                                                                                            // ready to grab L3 (already
                                                                                            // latched on L2) // 19.85
                        public static final double CLIMBER_L3_RETRACT_HEIGHT_INCHES = 9.5; // climber position to hover
                                                                                           // above L2 (scoring L3)
                                                                                           // (already latched on L3)
                                                                                           // // 9.5
                }
        }

        public static TunableNumber shooterMPStoRPM = new TunableNumber("shooterMPStoRPM", 1.150000);
        public static TunableNumber shooterOffset = new TunableNumber("shooterAngleOffset", 2.7190000000000);

        public static double shooterMPSToRPM(double mps) {

                return (151.0 * mps - 183.0) * 0.70;
                // return 3621.1-11.9904*Math.sqrt(76609-8340*mps);
        }

        public static Rotation2d launchAngleToHoodAngle(Rotation2d launchAngle, double rpm) {
                return Rotation2d.fromDegrees(0)
                                .plus(launchAngle);
                // return Rotation2d.fromDegrees(launchAngle.getDegrees() +
                // launchAngleOffset.get());
        }

        // PID constants
        public static final class PIDConstants {
                public static final class Turret {
                        // Position PID
                        public static final double kP0 = 140.0;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 0.3;
                        public static final double kV0 = 0.0;

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
                        public static final double kP0 = 0.3;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 0.3;
                        public static final double kV0 = 0.16;
                }

                public static final class Feeder {
                        public static final double kP0 = 2.4;
                        public static final double kI0 = 0.0;
                        public static final double kD0 = 0.0;
                        public static final double kS0 = 0.4;
                        public static final double kV0 = 0.5;

                        public static final double kP1 = 9999999.0;
                }
        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {

                public static final String LIMELIGHT_NAME = "limelight-turret";

                // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
                // radians

                public static final Translation3d LIMELIGHT_TO_TURRET_OFFSET = new Translation3d(
                                inchesToMeters(-6.08), inchesToMeters(0.0), inchesToMeters(7.9));

                // inchesToMeters(-6.75), inchesToMeters(0.0), inchesToMeters(27.75 - 17.8125));
                public static final Rotation3d LIMELIGHT_ROTATION_RELATIVE_TO_TURRET = new Rotation3d(
                                Math.toRadians(0.0),
                                Math.toRadians(-26.0),
                                Math.toRadians(3.5));

                public static final Transform3d turretToLimelight = new Transform3d(LIMELIGHT_TO_TURRET_OFFSET,
                                LIMELIGHT_ROTATION_RELATIVE_TO_TURRET);
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

                public static void updateLimelightPoseFromTurret(Pose3d robotToTurret, Rotation2d turretAngle,
                                Transform3d turretToCam,
                                String limelightName) {

                        Pose3d robotToCam = robotToTurret.transformBy(new Transform3d(
                                        Translation3d.kZero, new Rotation3d(turretAngle))).transformBy(turretToCam);

                        Logger.recordOutput("Constants/Vision/RobotToCam/X",
                                        Units.metersToInches(robotToCam.getX()));
                        Logger.recordOutput("Constants/Vision/RobotToCam/Y",
                                        -Units.metersToInches(robotToCam.getY()));
                        Logger.recordOutput("Constants/Vision/RobotToCam/Z",
                                        Units.metersToInches(robotToCam.getZ()));
                        Logger.recordOutput("Constants/Vision/RobotToCam/RX",
                                        Math.toDegrees(robotToCam.getRotation().getX()));
                        Logger.recordOutput("Constants/Vision/RobotToCam/RY",
                                        -Math.toDegrees(robotToCam.getRotation().getY()));
                        Logger.recordOutput("Constants/Vision/RobotToCam/RZ",
                                        Math.toDegrees(robotToCam.getRotation().getZ()));

                        try {
                                LimelightHelpers.setCameraPose_RobotSpace(
                                                limelightName,
                                                robotToCam.getX(),
                                                -robotToCam.getY(),
                                                robotToCam.getZ(),
                                                Math.toDegrees(robotToCam.getRotation().getX()),
                                                Math.toDegrees(-robotToCam.getRotation().getY()),
                                                Math.toDegrees(robotToCam.getRotation().getZ()));
                        } catch (Exception e) {
                                System.out.println("Could not set limelight pose: " + e.getMessage());
                        }
                }

                public static double getLimelightAngVelRelToField(double turretVel, double robotVel) {
                        return (turretVel + robotVel);
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
                        public static final double FLYWHEEL_GEAR_RATIO = 18.0 / 14.0; // 14 on motor
                        public static final double HOOD_ENCODER_TO_MECHANISM_GEAR_RATIO = 48.0 / 20.0; // encoder on 20
                        public static final double HOOD_GEAR_RATIO = (36.0 / 12.0) * (48.0 / 16.0) * (300.0 / 16.0);
                        public static final double HOOD_MOTOR_TO_ENCODER_GEAR_RATIO = HOOD_GEAR_RATIO
                                        / HOOD_ENCODER_TO_MECHANISM_GEAR_RATIO; // old shooter
                        public static final double TURRET_GEAR_RATIO = (60.0 / 12.0) * (134.0 / 15.0);
                        // public static final double TURRET_GEAR_RATIO = 6812.0 / 180.0;
                        // public static final double TURRET_GEAR_RATIO = 40.23809523809523;
                        // public static final double TURRET_GEAR_RATIO = 43.112;
                        // public static final double TURRET_GEAR_RATIO = 50.55;
                }

                public static final class Intake {
                        public static final double INTAKE_PIVOT_GEAR_RATIO = 1.0;
                        public static final double INTAKE_ROLLER_GEAR_RATIO = 1.0;
                }

                public static final class Feeder {
                        public static final double DYE_ROTOR_GEAR_RATIO = (48.0 / 10.0) * (130.0 / 18.0);
                }

                public static final class Climber {
                        public static final double CLIMBER_MOTOR_INCHES_PER_ROTATION = 0.25287202867;
                        public static final double CLIMBER_MAX_INCHES = 20.48;
                        public static final double CLIMBER_MAX_ROTATIONS = CLIMBER_MAX_INCHES
                                        / CLIMBER_MOTOR_INCHES_PER_ROTATION;
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
                public static final int HOOD_CANCODER_ID = 7;
                public static final int TURRET_CANCODER_ONE_ID = 5; // on driving pulley
                public static final int TURRET_CANCODER_TWO_ID = 6;

                // Intake
                public static final int INTAKE_PIVOT_MOTOR_ID = 13;
                public static final int INTAKE_ROLLER_MASTER_MOTOR_ID = 14;
                public static final int INTAKE_ROLLER_FOLLOWER_MOTOR_ID = 16;

                // Feeder
                public static final int DYE_ROTOR_MOTOR_ID = 15;

                // Climber
                public static final int CLIMBER_MASTER_MOTOR_ID = 19;
                public static final int CLIMBER_SLAVE_MOTOR_ID = 20;
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

        public static Pose2d flipFieldSide(Pose2d pose) {
                return new Pose2d(Constants.Physical.FIELD_LENGTH - pose.getX(),
                                Constants.Physical.FIELD_WIDTH - pose.getY(),
                                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
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
                return degrees * Math.PI / 180.0;
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
