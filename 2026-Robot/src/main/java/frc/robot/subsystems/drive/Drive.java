package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {

  DriveIO io;
  Peripherals peripherals;
  boolean firstClimb = false;

  // odometry

  private double m_initTime;
  private double m_currentTime;

  // Creating my kinematics object using the module locations

  double initAngle;
  double setAngle;
  double diffAngle;

  // path following PID values
  private double kXP = 4.00;
  private double kXI = 0.00;
  private double kXD = 1.20;

  private double kYP = kXP;
  private double kYI = kXI;
  private double kYD = kXD;

  private double kThetaP = 2.90;
  private double kThetaI = 0.00;
  private double kThetaD = 2.00;

  // auto placement PID values
  private double kkXP = 5.50;
  private double kkXI = 0.00;
  private double kkXD = 1.60;

  private double kkYP = kkXP;
  private double kkYI = kkXI;
  private double kkYD = kkXD;

  private double kkThetaP = 1.79;
  private double kkThetaI = 0.00;
  private double kkThetaD = 0.971;

  // l4 pid values
  private double kkXP4 = 4.30;
  private double kkXI4 = 0.00;
  private double kkXD4 = 1.70;

  private double kkYP4 = kkXP4;
  private double kkYI4 = kkXI4;
  private double kkYD4 = kkXD4;

  private double kkThetaP4 = 2.056;
  private double kkThetaI4 = 0.00;
  private double kkThetaD4 = 0.971;

  // l4 auto pids
  private double scalar = 0.9;
  private double kkXP4A = 4.30 * scalar;
  private double kkXI4A = 0.00 * scalar;
  private double kkXD4A = 1.70 * scalar;

  private double kkYP4A = kkXP4A;
  private double kkYI4A = kkXI4A;
  private double kkYD4A = kkXD4A;

  private double kkThetaP4A = 2.056;
  private double kkThetaI4A = 0.00;
  private double kkThetaD4A = 0.971;

  // l23 pid values
  private double kkXP23 = 4.00;
  private double kkXI23 = 0.00;
  private double kkXD23 = 1.60;

  private double kkYP23 = kkXP23;
  private double kkYI23 = kkXI23;
  private double kkYD23 = kkXD23;

  private double kkThetaP23 = 2.481;
  private double kkThetaI23 = 0.00;
  private double kkThetaD23 = 0.971;

  // l1 pid values
  private double kkXP1 = 4.00;
  private double kkXI1 = 0.00;
  private double kkXD1 = 1.40;

  private double kkYP1 = kkXP1;
  private double kkYI1 = kkXI1;
  private double kkYD1 = kkXD1;

  private double kkThetaP1 = 3.005;
  private double kkThetaI1 = 0.00;
  private double kkThetaD1 = 0.971;

  // Piece pickup values
  private double kkkXPPickup = 3.30;
  private double kkkXIPickup = 0.00;
  private double kkkXDPickup = 1.70;

  private double kkkYPPickup = kkkXPPickup;
  private double kkkYIPickup = kkkXIPickup;
  private double kkkYDPickup = kkkXDPickup;

  private double kkkThetaPPickup = 2.056;
  private double kkkThetaIPickup = 0.00;
  private double kkkThetaDPickup = 0.971;

  // teleop targeting PID values
  private double kTurningP = 0.04;
  private double kTurningI = 0;
  private double kTurningD = 0.06;
  private double kRotateP = 0.04;
  private double kRotateI = 0.0;
  private double kRotateD = 0.06;

  private PID xxPID = new PID(kkXP, kkXI, kkXD);
  private PID yyPID = new PID(kkYP, kkYI, kkYD);
  private PID thetaaPID = new PID(kkThetaP, kkThetaI, kkThetaD);

  private PID xxPID4 = new PID(kkXP4, kkXI4, kkXD4);
  private PID yyPID4 = new PID(kkYP4, kkYI4, kkYD4);
  private PID thetaaPID4 = new PID(kkThetaP4, kkThetaI4, kkThetaD4);

  private PID xxPID4A = new PID(kkXP4A, kkXI4A, kkXD4A);
  private PID yyPID4A = new PID(kkYP4A, kkYI4A, kkYD4A);
  private PID thetaaPID4A = new PID(kkThetaP4A, kkThetaI4A, kkThetaD4A);

  private PID xxPID23 = new PID(kkXP23, kkXI23, kkXD23);
  private PID yyPID23 = new PID(kkYP23, kkYI23, kkYD23);
  private PID thetaaPID23 = new PID(kkThetaP23, kkThetaI23, kkThetaD23);

  private PID xxPID1 = new PID(kkXP1, kkXI1, kkXD1);
  private PID yyPID1 = new PID(kkYP1, kkYI1, kkYD1);
  private PID thetaaPID1 = new PID(kkThetaP1, kkThetaI1, kkThetaD1);

  private PID xxPIDPickup = new PID(kkkXPPickup, kkkXIPickup, kkkXDPickup);
  private PID yyPIDPickup = new PID(kkkYPPickup, kkkYIPickup, kkkYDPickup);
  private PID thetaaPIDPickup = new PID(kkkThetaPPickup, kkkThetaIPickup, kkkThetaDPickup);

  private PID xPID = new PID(kXP, kXI, kXD);
  private PID yPID = new PID(kYP, kYI, kYD);
  private PID thetaPID = new PID(kThetaP, kThetaI, kThetaD);
  private PID turningPID = new PID(kTurningP, kTurningI, kTurningD);
  private PID rotatePID = new PID(kRotateP, kRotateI, kRotateD);

  private String m_fieldSide = "blue";

  AprilTagFieldLayout aprilTagFieldLayout;
  double startX;
  double startY;

  public boolean algaeMode = false;

  public enum DriveState {
    DEFAULT,
    IDLE,
    STOP,
    REEF,
    REEF_MORE,
    BACK,
    L3_REEF,
    L4_REEF,
    ALGAE,
    ALGAE_MORE,
    ALGAE_MORE_MORE,
    PROCESSOR,
    PROCESSOR_MORE,
    NET,
    NET_MORE,
    FEEDER,
    SCORE_L23,
    FEEDER_ALIGN,
    AUTO_FEEDER,
    AUTO_L1,
    AUTO_L1_MORE,
    FEEDER_AUTO,
    PIECE_PICKUP,
    AUTO_CLIMB
  }

  private DriveState wantedState = DriveState.IDLE;
  private DriveState systemState = DriveState.IDLE;

  /**
   * Creates a new instance of the Swerve Drive subsystem.
   * Initializes the Swerve Drive subsystem with the provided peripherals.
   * 
   * @param peripherals The peripherals used by the Swerve Drive subsystem.
   */
  Elevator elevator;

  public Drive(Peripherals peripherals, Elevator elevator) {
    this.peripherals = peripherals;
    this.elevator = elevator;
    if (RobotBase.isReal()) {
      this.io = new DriveIOComp(this.peripherals);
    } else {
      this.io = new DriveIOSim();
    }
  }

  // public boolean atSetpoint() {
  // double currentAngle = gyro.getYawDegrees();
  // if (getFieldSide().equals("red")) {
  // currentAngle -= 180;
  // }
  // return (Math.abs(Constants.standardizeAngleDegrees(currentAngle)
  // - Constants.standardizeAngleDegrees(angleSetpoint)) < 2);
  // }

  public void setWantedState(DriveState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(DriveState wantedState, double angle) {
    this.wantedState = wantedState;
  }

  /**
   * Initializes the robot with the specified field side configuration.
   * It sets up configurations when run on robot initialization, such as setting
   * the field side,
   * initializing each swerve module, configuring motor inversions, and setting
   * PID controller output limits.
   * Additionally, it sets the default command to the DriveDefault command.
   *
   * @param fieldSide The side of the field (e.g., "red" or "blue").
   */
  public void init(String fieldSide) {
    // sets configurations when run on robot initalization
    this.m_fieldSide = fieldSide;

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().getPath() + "/" + "2025-reefscape-andymark.json");
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
    }
    xxPID.setMinOutput(-3.0);
    xxPID.setMaxOutput(3.0);

    yyPID.setMinOutput(-3.0);
    yyPID.setMaxOutput(3.0);

    thetaaPID.setMinOutput(-3.0);
    thetaaPID.setMaxOutput(3.0);

    xxPID4.setMinOutput(-2.0);
    xxPID4.setMaxOutput(2.0);

    yyPID4.setMinOutput(-2.0);
    yyPID4.setMaxOutput(2.0);

    thetaaPID4.setMinOutput(-2.0);
    thetaaPID4.setMaxOutput(2.0);

    xxPID4A.setMinOutput(-1.4);
    xxPID4A.setMaxOutput(1.4);

    yyPID4A.setMinOutput(-1.4);
    yyPID4A.setMaxOutput(1.4);

    thetaaPID4A.setMinOutput(-1.4);
    thetaaPID4A.setMaxOutput(1.4);

    xxPID23.setMinOutput(-4.0);
    xxPID23.setMaxOutput(4.0);

    yyPID23.setMinOutput(-4.0);
    yyPID23.setMaxOutput(4.0);

    thetaaPID23.setMinOutput(-4.0);
    thetaaPID23.setMaxOutput(4.0);

    xxPID1.setMinOutput(-4.0);
    xxPID1.setMaxOutput(4.0);

    yyPID1.setMinOutput(-4.0);
    yyPID1.setMaxOutput(4.0);

    thetaaPID1.setMinOutput(-4.0);
    thetaaPID1.setMaxOutput(4.0);

    xxPIDPickup.setMinOutput(-1.0);
    xxPIDPickup.setMaxOutput(1.0);

    yyPIDPickup.setMinOutput(-1.0);
    yyPIDPickup.setMaxOutput(1.0);

    thetaaPIDPickup.setMinOutput(-2.0);
    thetaaPIDPickup.setMaxOutput(2.0);

    xPID.setMinOutput(-4.9);
    xPID.setMaxOutput(4.9);

    yPID.setMinOutput(-4.9);
    yPID.setMaxOutput(4.9);

    thetaPID.setMinOutput(-3);
    thetaPID.setMaxOutput(3);

    turningPID.setMinOutput(-3);
    turningPID.setMaxOutput(3);

    rotatePID.setMinOutput(-2);
    rotatePID.setMaxOutput(2);
  }

  public void teleopInit() {

  }

  /**
   * Zeros the IMU (Inertial Measurement Unit) mid-match and resets the odometry
   * with a zeroed angle.
   * It resets the angle reported by the pigeon sensor to zero and updates the
   * odometry with this new zeroed angle.
   */
  public void zeroIMU() {
    io.zeroIMU();
  }

  /**
   * Adjusts the angle reported by the pigeon sensor after an autonomous routine.
   * It adds 180 degrees to the current angle reported by the pigeon sensor and
   * wraps it around 360 degrees.
   */
  public void setPigeonAfterAuto() {
    io.setYaw((io.getYaw().getDegrees() + 180) % 360);
  }

  /**
   * Sets the angle reported by the pigeon sensor to the specified value.
   *
   * @param angle The angle to set for the pigeon sensor in degrees.
   */
  public void setPigeonAngle(double angle) {
    io.setYaw(angle);
  }

  /**
   * Retrieves the current angle reported by the pigeon sensor.
   *
   * @return The current angle reported by the pigeon sensor in degrees.
   */
  public double getPigeonAngle() {
    return io.getYaw().getDegrees();
  }

  /**
   * Sets the PID values for all swerve modules to zero, keeping the wheels
   * straight.
   */
  public void setWheelsStraight() {

  }

  /**
   * Initializes the robot's state for autonomous mode based on the provided path
   * points.
   * 
   * @param pathPoints The array of path points representing the trajectory for
   *                   the autonomous routine.
   */
  public void autoInit(JSONArray pathPoints) {
    // runs at start of autonomous
    java.util.logging.Logger.getGlobal().info("Auto init");
    JSONObject firstPoint = pathPoints.getJSONObject(0);
    double firstPointX = firstPoint.getDouble("x");
    double firstPointY = firstPoint.getDouble("y");
    double firstPointAngle = firstPoint.getDouble("angle");

    // changing odometry if on red side, don't need to change y because it will be
    // the same for autos on either side
    if (this.m_fieldSide == "blue") {
      firstPointX = Constants.Physical.FIELD_LENGTH - firstPointX;
      firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
      firstPointAngle = Math.PI + firstPointAngle;
    }

    if (OI.isProcessorSide()) {
      firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
      firstPointAngle = -firstPointAngle;
    }
    Pose2d firstPose2d = new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle));
    io.setCurrentLimits(60, 120);
    io.setPosition(firstPose2d);

    m_initTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets the current field side designation.
   * 
   * @param side The field side designation to set, indicating whether the robot
   *             is positioned on the "blue" or "red" side of the field.
   */
  public void setFieldSide(String side) {
    m_fieldSide = side;
  }

  /**
   * Retrieves the current field side designation.
   * 
   * @return The current field side designation, indicating whether the robot is
   *         positioned on the "blue" or "red" side of the field.
   */
  public String getFieldSide() {
    return m_fieldSide;
  }

  /**
   * Retrieves the current timestamp relative to the start of the robot operation.
   * 
   * @return The current timestamp in seconds since the start of the robot
   *         operation.
   */
  public double getCurrentTime() {
    return m_currentTime;
  }

  public void setOdometry(Pose2d pose) {
    // Logger.recordOutput("Odometry Reset to:", pose.toString());
    java.util.logging.Logger.getGlobal().fine("New Odometry Pose: " + pose.toString());
    io.setPosition(pose);
  }

  public boolean isPoseInField(Pose2d pose) {
    if (pose.getY() < 0 || pose.getY() > Constants.Physical.FIELD_WIDTH || pose.getX() < 0
        || pose.getX() > Constants.Physical.FIELD_LENGTH) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * Retrieves the states of the modules (position and ground speed) of the
   * robot's swerve drive system.
   *
   * @return An array containing the states of each wheel module, consisting of:
   *         front right module position in degrees,
   *         front right module ground speed in meters per second,
   *         front left module position in degrees,
   *         front left module ground speed in meters per second,
   *         back left module position in degrees,
   *         back left module ground speed in meters per second,
   *         back right module position in degrees,
   *         back right module ground speed in meters per second.
   */
  // public double[] getModuleStates() {
  // double[] states = {
  // frontLeft.getCanCoderPosition() * 360.0, frontLeft.getGroundSpeed(),
  // frontRight.getCanCoderPosition() * 360.0, frontRight.getGroundSpeed(),
  // backLeft.getCanCoderPosition() * 360.0, backLeft.getGroundSpeed(),
  // backRight.getCanCoderPosition() * 360.0, backRight.getGroundSpeed(),
  // };
  // return states;
  // }

  /**
   * Retrieves the setpoints of the modules (angle and drive motors) of the
   * robot's swerve drive system.
   *
   * @return An array containing the setpoints of the angle and drive motors for
   *         each wheel module, in the order:
   *         front right angle motor, front right drive motor,
   *         front left angle motor, front left drive motor,
   *         back left angle motor, back left drive motor,
   *         back right angle motor, back right drive motor.
   */
  // public double[] getModuleSetpoints() {
  // double[] setpoints = {
  // frontLeft.getAngleMotorSetpoint() * 360, frontLeft.getDriveMotorSetpoint(),
  // frontRight.getAngleMotorSetpoint() * 360, frontRight.getDriveMotorSetpoint(),
  // backLeft.getAngleMotorSetpoint() * 360, backLeft.getDriveMotorSetpoint(),
  // backRight.getAngleMotorSetpoint() * 360, backRight.getDriveMotorSetpoint(),
  // };
  // return setpoints;
  // }

  private boolean autoPlacingFront = true;

  public boolean getAutoPlacementSideIsFront() {
    return autoPlacingFront;
  }

  public double getAngleDifferenceDegrees(double angle1, double angle2) {
    double difference = Math.abs(angle1 - angle2) % 360;
    return difference > 180 ? 360 - difference : difference;
  }

  public boolean isGoingForL3Algae() {
    double thetaSetpoint = getAlgaeClosestSetpoint(getMt2Pose2d()).getRotation().getRadians();
    if (autoPlacingFront) {
      if (!isOnBlueSide()) {
        if (getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint), 60.0) <= 30.0 || getAngleDifferenceDegrees(
            Math.toDegrees(thetaSetpoint), 180.0) <= 30.0 || getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint),
                300.0) <= 30) {
          return true;
        } else {
          return false;
        }
      } else {
        if (getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint), 0.0) <= 30.0 || getAngleDifferenceDegrees(
            Math.toDegrees(thetaSetpoint), 120.0) <= 30.0 || getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint),
                240.0) <= 30) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      if (!isOnBlueSide()) {
        if (getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint), 60.0) <= 30.0 || getAngleDifferenceDegrees(
            Math.toDegrees(thetaSetpoint), 180.0) <= 30.0 || getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint),
                300.0) <= 30) {
          return false;
        } else {
          return true;
        }
      } else {
        if (getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint), 0.0) <= 30.0 || getAngleDifferenceDegrees(
            Math.toDegrees(thetaSetpoint), 120.0) <= 30.0 || getAngleDifferenceDegrees(Math.toDegrees(thetaSetpoint),
                240.0) <= 30) {
          return false;
        } else {
          return true;
        }
      }
    }
  }

  public Pose2d getAlgaeMoreClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (!isOnBlueSide()) {
      for (int i = 0; i < Constants.Reef.algaeRedFrontPlacingPositionsMore.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositionsMore.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositionsMore.get(i).getY()),
        // 2));
        currentDist = Math.hypot(
            x - Constants.Reef.algaeRedFrontPlacingPositionsMore.get(i).getX(),
            y - Constants.Reef.algaeRedFrontPlacingPositionsMore.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeRedFrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeRedFrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeRedBackPlacingPositionsMore.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.algaeBlueFrontPlacingPositionsMore.size(); i++) {
        currentDist = Math.hypot(
            x - Constants.Reef.algaeBlueFrontPlacingPositionsMore.get(i).getX(),
            y - Constants.Reef.algaeBlueFrontPlacingPositionsMore.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeBlueFrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeBlueFrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeBlueBackPlacingPositionsMore.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      return chosenSetpoint;
    }
  }

  public Pose2d getAlgaeMoreMoreClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(Math.toDegrees(currentOdometry.getRotation().getRadians()));
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (!isOnBlueSide()) {
      for (int i = 0; i < Constants.Reef.algaeRedFrontPlacingPositionsMoreMore.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositionsMoreMore.get(i).getX()), 2)
        // + Math.pow((y -
        // Constants.Reef.redFrontPlacingPositionsMoreMore.get(i).getY()),
        // 2));
        currentDist = Math.hypot(
            x - Constants.Reef.algaeRedFrontPlacingPositionsMoreMore.get(i).getX(),
            y - Constants.Reef.algaeRedFrontPlacingPositionsMoreMore.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeRedFrontPlacingPositionsMoreMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeRedFrontPlacingPositionsMoreMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeRedBackPlacingPositionsMoreMore.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.algaeBlueFrontPlacingPositionsMoreMore.size(); i++) {
        currentDist = Math.hypot(
            x - Constants.Reef.algaeBlueFrontPlacingPositionsMoreMore.get(i).getX(),
            y - Constants.Reef.algaeBlueFrontPlacingPositionsMoreMore.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeBlueFrontPlacingPositionsMoreMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeBlueFrontPlacingPositionsMoreMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeBlueBackPlacingPositionsMoreMore.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      return chosenSetpoint;
    }
  }

  public Pose2d getAlgaeClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (!isOnBlueSide()) {
      for (int i = 0; i < Constants.Reef.algaeRedFrontPlacingPositions.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            x - Constants.Reef.algaeRedFrontPlacingPositions.get(i).getX(),
            y - Constants.Reef.algaeRedFrontPlacingPositions.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeRedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeRedFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeRedBackPlacingPositions.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.algaeBlueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - Constants.Reef.algaeBlueFrontPlacingPositions.get(i).getX(),
            y - Constants.Reef.algaeBlueFrontPlacingPositions.get(i).getY());
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.algaeBlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.algaeBlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.algaeBlueBackPlacingPositions.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      return chosenSetpoint;
    }
  }

  public Pose2d getReefClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */, boolean notClosest) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.redFrontPlacingPositions.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            x - (Constants.Reef.redFrontPlacingPositions.get(i).getX() + Constants.Reef.redBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.redFrontPlacingPositions.get(i).getY()
                + Constants.Reef.redBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.redFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.redFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.redBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation()
              .getDistance(Constants.Reef.redFrontPlacingPositions.get(i).getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.redFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY() - Constants.Reef.redFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.redBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.redBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.redFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.redBackPlacingPositions
                      .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.redFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.redFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.redBackPlacingPositions.get(i);
            }
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.blueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.blueFrontPlacingPositions.get(i).getX() + Constants.Reef.blueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.blueFrontPlacingPositions.get(i).getY() + Constants.Reef.blueBackPlacingPositions.get(i)
                .getY()) / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.blueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.blueBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation().getDistance(Constants.Reef.blueFrontPlacingPositions.get(i)
              .getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.blueFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(
                          origionalSetpointPose.getY() - Constants.Reef.blueFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.blueBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.blueBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.blueFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.blueBackPlacingPositions
                      .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.blueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.blueFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.blueBackPlacingPositions.get(i);
            }
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L2 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getL1ReefClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.redL1FrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.redL1BackPlacingPositions.get(i).getX() + Constants.Reef.redL1BackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.redL1BackPlacingPositions.get(i).getY()
                + Constants.Reef.redL1BackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.redL1FrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.redL1FrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.redL1BackPlacingPositions.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.blueL1FrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.blueL1BackPlacingPositions.get(i).getX() + Constants.Reef.blueL1BackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.blueL1BackPlacingPositions.get(i).getY()
                + Constants.Reef.blueL1BackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueL1FrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.blueL1FrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.blueL1BackPlacingPositions.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L2 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getL1ReefClosestSetpointMore(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.redL1FrontPlacingPositionsMore.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redL1FrontPlacingPositionsMore.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redL1FrontPlacingPositionsMore.get(i).getY()),
        // 2));
        currentDist = Math.hypot(
            x - (Constants.Reef.redL1BackPlacingPositionsMore.get(i).getX()
                + Constants.Reef.redL1BackPlacingPositionsMore
                    .get(i)
                    .getX())
                / 2,
            y - (Constants.Reef.redL1BackPlacingPositionsMore.get(i).getY()
                + Constants.Reef.redL1BackPlacingPositionsMore.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.redL1FrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.redL1FrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.redL1BackPlacingPositionsMore.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.blueL1FrontPlacingPositionsMore.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.blueL1BackPlacingPositionsMore.get(i).getX()
                + Constants.Reef.blueL1BackPlacingPositionsMore
                    .get(i)
                    .getX())
                / 2,
            y - (Constants.Reef.blueL1BackPlacingPositionsMore.get(i).getY()
                + Constants.Reef.blueL1BackPlacingPositionsMore.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueL1FrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.blueL1FrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.blueL1BackPlacingPositionsMore.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L2 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getReefMoreClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.redFrontPlacingPositionsMore.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.redFrontPlacingPositionsMore.get(i).getX() + Constants.Reef.redBackPlacingPositionsMore
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.redFrontPlacingPositionsMore.get(i).getY()
                + Constants.Reef.redBackPlacingPositionsMore.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.redFrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.redFrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.redBackPlacingPositionsMore.get(i);
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.blueFrontPlacingPositionsMore.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.blueFrontPlacingPositionsMore.get(i).getX()
                + Constants.Reef.blueBackPlacingPositionsMore
                    .get(i)
                    .getX())
                / 2,
            y - (Constants.Reef.blueFrontPlacingPositionsMore.get(i).getY()
                + Constants.Reef.blueBackPlacingPositionsMore.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueFrontPlacingPositionsMore.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.blueFrontPlacingPositionsMore.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.blueBackPlacingPositionsMore.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 2.0) {
      return getMt2Pose2d();
    } else {
      return chosenSetpoint;
    }
  }

  public Pose2d getReefL44ClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */, boolean notClosest) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));

    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.l4RedFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l4RedFrontPlacingPositions.get(i).getX() + Constants.Reef.l4RedBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l4RedFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l4RedBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l4RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l4RedFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l4RedBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation().getDistance(Constants.Reef.l4RedFrontPlacingPositions.get(i)
              .getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l4RedFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l4RedFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.l4RedBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l4RedBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4RedFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4RedBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l4RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l4RedFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l4RedBackPlacingPositions.get(i);
            }
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.l4BlueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l4BlueFrontPlacingPositions.get(i).getX() + Constants.Reef.l4BlueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l4BlueFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l4BlueBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l4BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l4BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l4BlueBackPlacingPositions.get(i);
          }
        } else if (notClosest
            && ((Math
                .abs(origionalSetpointPose.getX() - Constants.Reef.l4BlueFrontPlacingPositions.get(i).getX()) < 0.01
                || Math
                    .abs(
                        origionalSetpointPose.getY() - Constants.Reef.l4BlueFrontPlacingPositions.get(i).getY()) < 0.01)
                && (Math
                    .abs(origionalSetpointPose.getX() - Constants.Reef.l4BlueBackPlacingPositions.get(i).getX()) < 0.01
                    || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l4BlueBackPlacingPositions.get(i)
                        .getY()) < 0.01))
            && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4BlueFrontPlacingPositions
                .get(i).getRotation().getRadians()) < 0.01
                || Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4BlueBackPlacingPositions
                    .get(i).getRotation().getRadians()) < 0.01)) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l4BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l4BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l4BlueBackPlacingPositions.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L4 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getReefL3ClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */, boolean notClosest) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.l3RedFrontPlacingPositions.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            x - (Constants.Reef.l3RedFrontPlacingPositions.get(i).getX() + Constants.Reef.l3RedBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l3RedFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l3RedBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l3RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l3RedFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l3RedBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation().getDistance(Constants.Reef.l3RedFrontPlacingPositions.get(i)
              .getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l3RedFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l3RedFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.l3RedBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l3RedBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3RedFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3RedBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l3RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l3RedFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l3RedBackPlacingPositions.get(i);
            }
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.l3BlueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l3BlueFrontPlacingPositions.get(i).getX() + Constants.Reef.l3BlueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l3BlueFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l3BlueBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l3BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l3BlueBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((Math.hypot(
              origionalSetpointPose.getX() - Constants.Reef.l3BlueFrontPlacingPositions.get(i)
                  .getX(),
              origionalSetpointPose.getY() - Constants.Reef.l3BlueFrontPlacingPositions.get(i)
                  .getY()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l3BlueFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l3BlueFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(
                          origionalSetpointPose.getX() - Constants.Reef.l3BlueBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l3BlueBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3BlueFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3BlueBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l3BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l3BlueFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l3BlueBackPlacingPositions.get(i);
            }
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L3 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getReefL33ClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */, boolean notClosest) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));

    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.l3RedFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l3RedFrontPlacingPositions.get(i).getX() + Constants.Reef.l3RedBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l3RedFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l3RedBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l3RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l3RedFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l3RedBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation().getDistance(Constants.Reef.l3RedFrontPlacingPositions.get(i)
              .getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l3RedFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l3RedFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.l3RedBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l3RedBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3RedFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3RedBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l3RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l3RedFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l3RedBackPlacingPositions.get(i);
            }
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.l3BlueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l3BlueFrontPlacingPositions.get(i).getX() + Constants.Reef.l3BlueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l3BlueFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l3BlueBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l3BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l3BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l3BlueBackPlacingPositions.get(i);
          }
        } else if (notClosest
            && ((Math
                .abs(origionalSetpointPose.getX() - Constants.Reef.l3BlueFrontPlacingPositions.get(i).getX()) < 0.01
                || Math
                    .abs(
                        origionalSetpointPose.getY() - Constants.Reef.l3BlueFrontPlacingPositions.get(i).getY()) < 0.01)
                && (Math
                    .abs(origionalSetpointPose.getX() - Constants.Reef.l3BlueBackPlacingPositions.get(i).getX()) < 0.01
                    || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l3BlueBackPlacingPositions.get(i)
                        .getY()) < 0.01))
            && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3BlueFrontPlacingPositions
                .get(i).getRotation().getRadians()) < 0.01
                || Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l3BlueBackPlacingPositions
                    .get(i).getRotation().getRadians()) < 0.01)) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l3BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l3BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l3BlueBackPlacingPositions.get(i);
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L3 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  public Pose2d getReefL4ClosestSetpoint(Pose2d currentOdometry /* {x, y, thetaRadians} */, boolean notClosest) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.l4RedFrontPlacingPositions.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            x - (Constants.Reef.l4RedFrontPlacingPositions.get(i).getX() + Constants.Reef.l4RedBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l4RedFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l4RedBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.l4RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l4RedFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l4RedBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((origionalSetpointPose.getTranslation().getDistance(Constants.Reef.l4RedFrontPlacingPositions.get(i)
              .getTranslation()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l4RedFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l4RedFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(origionalSetpointPose.getX() - Constants.Reef.l4RedBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l4RedBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4RedFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4RedBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l4RedFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l4RedFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l4RedBackPlacingPositions.get(i);
            }
          }
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.l4BlueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.l4BlueFrontPlacingPositions.get(i).getX() + Constants.Reef.l4BlueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.l4BlueFrontPlacingPositions.get(i).getY()
                + Constants.Reef.l4BlueBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist && !notClosest) {
          dist = currentDist;
          if (getAngleDifferenceDegrees(theta,
              Constants.Reef.blueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            chosenSetpoint = Constants.Reef.l4BlueFrontPlacingPositions.get(i);
          } else {
            autoPlacingFront = false;
            chosenSetpoint = Constants.Reef.l4BlueBackPlacingPositions.get(i);
          }
        } else if (notClosest) {
          if ((Math.hypot(
              origionalSetpointPose.getX() - Constants.Reef.l4BlueFrontPlacingPositions.get(i)
                  .getX(),
              origionalSetpointPose.getY() - Constants.Reef.l4BlueFrontPlacingPositions.get(i)
                  .getY()) < 0.9)
              && ((Math
                  .abs(origionalSetpointPose.getX() - Constants.Reef.l4BlueFrontPlacingPositions.get(i).getX()) > 0.01
                  || Math
                      .abs(origionalSetpointPose.getY()
                          - Constants.Reef.l4BlueFrontPlacingPositions.get(i).getY()) > 0.01)
                  && (Math
                      .abs(
                          origionalSetpointPose.getX() - Constants.Reef.l4BlueBackPlacingPositions.get(i).getX()) > 0.01
                      || Math.abs(origionalSetpointPose.getY() - Constants.Reef.l4BlueBackPlacingPositions.get(i)
                          .getY()) > 0.01))
              && (Math.abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4BlueFrontPlacingPositions
                  .get(i).getRotation().getRadians()) < 0.01
                  || Math
                      .abs(origionalSetpointPose.getRotation().getRadians() - Constants.Reef.l4BlueBackPlacingPositions
                          .get(i).getRotation().getRadians()) < 0.01)) {
            dist = currentDist;
            if (getAngleDifferenceDegrees(theta,
                Constants.Reef.l4BlueFrontPlacingPositions.get(i).getRotation().getDegrees()) <= 90) {
              autoPlacingFront = true;
              chosenSetpoint = Constants.Reef.l4BlueFrontPlacingPositions.get(i);
            } else {
              autoPlacingFront = false;
              chosenSetpoint = Constants.Reef.l4BlueBackPlacingPositions.get(i);
            }
          }
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      Logger.recordOutput("L4 target pose", chosenSetpoint);
      return chosenSetpoint;
    }
  }

  boolean firstTimeAutoPickup = false;
  double firstTimePickupAngle = 0.0;
  boolean firstTimeCalculated = false;
  boolean firstTimeGoingInCalculated = false;
  boolean firstTimeGoingIn = false;
  boolean hasTrack = false;

  Pose2d targetPose = new Pose2d();
  Pose2d c1 = new Pose2d();
  Pose2d c2 = new Pose2d();

  public Pose2d getGamePiecePosition() {
    if (!firstTimeAutoPickup) {
      firstTimePickupAngle = getMt2Pose2dAngle();
      firstTimeAutoPickup = true;
    }

    double yaw = 0.0;
    double pitch = 0.0;
    var result = peripherals.getFrontGamePieceCamResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> tracks = result.getTargets();
      for (int i = 0; i < tracks.size(); i++) {
        int id = tracks.get(i).getDetectedObjectClassID();
        if (id == 0) {
          tracks.remove(i);
          i--;
        }
      }
      if (tracks.isEmpty()) {
        return new Pose2d();
      } else {
        double minPitch = tracks.get(0).getPitch();
        int index = 0;
        for (int i = 1; i < tracks.size(); i++) {
          int id = tracks.get(0).getDetectedObjectClassID();
          java.util.logging.Logger.getGlobal().finer("id: " + id);
          if (tracks.get(i).getPitch() < minPitch) {
            minPitch = tracks.get(i).getPitch();
            index = i;
          }
        }
        pitch = minPitch;
        yaw = tracks.get(index).getYaw();
      }
    }
    // double yFromIntake = 0.0;

    if (yaw != 0.0 && pitch != 0.0) {
      java.util.logging.Logger.getGlobal().finer("calculating%");
      double cameraYaw = 15.0;
      double limelightXOffset = Constants.inchesToMeters(2.25);
      double limelightYOffset = Constants.inchesToMeters(-11.5);
      double intakeXOffset = Constants.inchesToMeters(20.5);
      double intakeYOffset = Constants.inchesToMeters(3.0); // 4.0

      double robotX = getMt2Pose2dX();
      double robotY = getMt2Pose2dY();
      // double robotAngle = getMt2Pose2dAngle();
      double robotAngle = firstTimePickupAngle;
      Pose2d robotPose = new Pose2d(robotX, robotY, new Rotation2d(robotAngle));

      // double targetDistance = Math
      // .abs((limelightHeight - gamePieceHeight) /
      // Math.tan(Math.toRadians(-limelightAngle + pitch)));
      double targetDistance = Constants.inchesToMeters(Constants.Vision.getCoralDistanceFromPitch(pitch));
      Logger.recordOutput("Distance to Coral", targetDistance);
      double noteY = -(targetDistance * Math.sin(Math.toRadians(-cameraYaw + yaw)));
      double noteX = ((targetDistance * Math.cos(Math.toRadians(cameraYaw - yaw))));
      Logger.recordOutput("coral x", noteX);
      Logger.recordOutput("coral y", noteY);
      double xFromRobot = noteX + limelightXOffset;
      double yFromRobot = noteY + limelightYOffset;
      Pose2d coralPose = robotPose.transformBy(new Transform2d(xFromRobot, yFromRobot, new Rotation2d()));

      Logger.recordOutput("coral rc x", xFromRobot);
      Logger.recordOutput("coral rc y", yFromRobot);
      Logger.recordOutput("coral pose", coralPose);

      double xFromIntake = xFromRobot - intakeXOffset;
      double yFromIntake = yFromRobot - intakeYOffset;
      if (!firstTimeCalculated) {
        hasTrack = true;
        targetPose = robotPose.transformBy(new Transform2d(xFromIntake,
            yFromIntake, new Rotation2d()));
        Logger.recordOutput("coral intake x", xFromIntake);
        Logger.recordOutput("coral intake y", yFromIntake);
        double projectedTx = Math.atan2(Math.tan(Math.toRadians(yaw)), Math.cos(Math.toRadians(pitch)));
        java.util.logging.Logger.getGlobal().finer("Projected: " + Math.toDegrees(projectedTx));
        java.util.logging.Logger.getGlobal().finer("actual: " + yaw);
        Pose2d intakeFieldPose = robotPose.transformBy(new Transform2d(intakeXOffset, intakeYOffset, new Rotation2d()));
        double angleToPiece = Constants.standardizeAngleDegrees(Math.toDegrees(Math.atan2(
            intakeFieldPose.getY() - coralPose.getY(), intakeFieldPose.getX() - coralPose.getX()) - Math.PI));
        Logger.recordOutput("Angle to Piece", (angleToPiece));
        Logger.recordOutput("Intake field pose", intakeFieldPose);

        double deltaX = coralPose.getX() - intakeFieldPose.getX();
        double deltaY = coralPose.getY() - intakeFieldPose.getY();
        double targetAngle = Math.atan2(deltaY, deltaX);
        Pose2d poseFromRobotFront = new Pose2d(Constants.metersToInches(xFromRobot - Constants.inchesToMeters(16.5)),
            Constants.metersToInches(yFromRobot), new Rotation2d());
        Logger.recordOutput("Pose from robot front", poseFromRobotFront);

        // Adjust for the robot's current heading and camera yaw
        double requiredTurn = targetAngle - Math.toRadians(cameraYaw);

        // Normalize to range [-π, π] for minimal turning
        requiredTurn = (requiredTurn + (2 * Math.PI)) % (2 * Math.PI);
        // driveToPoint(targetPose.getX(), targetPose.getY(),
        Logger.recordOutput("new angle", Math.toDegrees(requiredTurn));
        // Pose2d targetPose = robotPose
        // .transformBy(new Transform2d(xFromIntake, yFromIntake, new
        // Rotation2d(requiredTurn)));
        Logger.recordOutput("target pickup", targetPose);

        double mx = (robotX + targetPose.getX()) / 2;
        double my = (robotY + targetPose.getY()) / 2;

        // Compute the length of AB (hypotenuse)
        double abLength = Math.sqrt(Math.pow(robotX - targetPose.getX(), 2) + Math.pow(robotY - targetPose.getY(), 2));

        // The expected right triangle height from midpoint to C
        double d = Math.sqrt(xFromIntake * xFromIntake + yFromIntake * yFromIntake - (abLength * abLength) / 2);

        // Compute the perpendicular direction (normalized)
        double perpX = -(robotY - targetPose.getY()) / abLength;
        double perpY = (robotX - targetPose.getX()) / abLength;

        // Compute the two possible C points
        double c1X = mx + d * perpX;
        double c1Y = my + d * perpY;

        double c2X = mx - d * perpX;
        double c2Y = my - d * perpY;
        c1 = new Pose2d(c1X, c1Y, new Rotation2d(robotAngle));
        c2 = new Pose2d(c2X, c2Y, new Rotation2d(robotAngle));
        Logger.recordOutput("c1x", c1);
        Logger.recordOutput("c1y", c2);
        firstTimeCalculated = true;
        java.util.logging.Logger.getGlobal().finer("Running once");
      }
      // targetPose.getRotation().getRadians());
      // if (){

      // } else
      // if (Math.abs(yFromIntake) < 0.2) {
      java.util.logging.Logger.getGlobal().finer("going in");
      // if (!firstTimeGoingInCalculated) {
      // firstTimeCalculated = false;
      // }
      firstTimeGoingIn = true;
      return targetPose;
      // } else if (yFromIntake < 0.0 && !firstTimeGoingIn) {
      // System.out.println("to the left");
      // return c1;
      // } else {
      // System.out.println("to the right");
      // return c2;
      // }
      // double pieceXFromIntake = noteX - intakeXOffset;
      // double pieceYFromIntake = noteY - intakeYOffset;
    } else

    {
      // System.out.println("default game piece");
      return new Pose2d();
    }
  }

  public double getGyroYaw() {
    return io.getYaw().getDegrees();
  }

  public void goToCoral() {
    double yaw = 0.0;
    double pitch = 0.0;
    var result = peripherals.getFrontGamePieceCamResult();
    List<PhotonTrackedTarget> tracks = new ArrayList<>(result.getTargets());
    List<PhotonTrackedTarget> coralTargets = new ArrayList<>();
    for (PhotonTrackedTarget track : tracks) {
      if (track.objDetectId == 1) {
        coralTargets.add(track);
      }
    }

    if (result.hasTargets() && !coralTargets.isEmpty()) {
      PhotonTrackedTarget bestTrack = coralTargets.get(0);
      yaw = bestTrack.getYaw();
      pitch = bestTrack.getPitch();
    }

    if (yaw != 0.0 && pitch != 0.0) {
      double currentAngle = -yaw;
      java.util.logging.Logger.getGlobal().finer("currentAngle: " + currentAngle);
      double wantedAngleInFrame = 14.7;
      double cameraToRobotAngleOffset = 35.0;
      rotatePID.setSetPoint(wantedAngleInFrame);
      rotatePID.updatePID(currentAngle);
      double r = -rotatePID.getResult();
      double driveAngleDeg = wantedAngleInFrame + cameraToRobotAngleOffset;
      double wantedSpeedMPS = 1.0;
      Vector v = new Vector(Math.cos(Math.toRadians(driveAngleDeg)), Math.sin(Math.toRadians(driveAngleDeg)))
          .scaled(wantedSpeedMPS);
      autoRobotCentricDrive(v, r);
    }
  }

  public Pose2d getMt2Pose2d() {
    return io.getPosition();
  }

  /**
   * Retrieves the current X-coordinate of the robot from odometry.
   *
   * @return The current X-coordinate of the robot.
   */
  public double getMt2Pose2dX() {
    return getMt2Pose2d().getX();
  }

  /**
   * Retrieves the current Y-coordinate of the robot from odometry.
   *
   * @return The current Y-coordinate of the robot.
   */
  public double getMt2Pose2dY() {
    return getMt2Pose2d().getY();
  }

  /**
   * Retrieves the current orientation angle of the robot from odometry.
   *
   * @return The current orientation angle of the robot in radians.
   */
  public double getMt2Pose2dAngle() {
    return getMt2Pose2d().getRotation().getRadians();
  }

  /**
   * Drives the robot with alignment adjustment based on the specified angle from
   * placement.
   * 
   * @param degreesFromPlacement The angle in degrees from the placement
   *                             orientation to align with.
   */
  public void driveAutoAligned(double degreesFromPlacement) {

    double turn = degreesFromPlacement;

    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

    if (Math.abs(originalX) < 0.05) {
      originalX = 0;
    }
    if (Math.abs(originalY) < 0.05) {
      originalY = 0;
    }

    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.drive(controllerVector, turn);
  }

  /**
   * Turns the robot in robot-centric mode.
   * 
   * @param turn The rate at which the robot should turn in radians per second.
   */
  public void autoRobotCentricTurn(double turn) {
    io.drive(new Vector(0, 0), turn);
  }

  /**
   * Drives the robot in robot-centric mode using velocity vector and turning
   * rate.
   * 
   * @param velocityVector    The velocity vector containing x and y velocities in
   *                          meters per second (m/s).
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoRobotCentricDrive(Vector velocityVector, double turnRadiansPerSec) {
    io.driveRobotCentric(velocityVector, turnRadiansPerSec);
  }

  /**
   * Drives the robot during teleoperation.
   * 
   * @apiNote This method updates the fused odometry array and controls the
   *          robot's movement based on joystick inputs.
   */
  public void teleopDrive() {
    double oiRX = OI.getDriverRightX();
    double oiLX = OI.getDriverLeftX();
    double oiRY = OI.getDriverRightY();
    double oiLY = OI.getDriverLeftY();
    if (OI.operatorLT.getAsBoolean() && OI.operatorRT.getAsBoolean()) {
      oiRX = OI.getOperatorRightX();
      oiLX = OI.getOperatorLeftX();
      oiRY = OI.getOperatorRightY();
      oiLY = OI.getOperatorLeftY();
    }
    double speedMultiplier = (((60 - Constants.metersToInches(elevator.getElevatorPosition())) * 0.4 / 50) + 0.6);

    if (elevator.getElevatorPosition() > Constants.inchesToMeters(10)) {
      oiRX = oiRX * speedMultiplier;
      oiLX = oiLX * speedMultiplier;
      oiRY = oiRY * speedMultiplier;
      oiLY = oiLY * speedMultiplier;
    }
    double turnLimit = 0.17;

    if (OI.driverController.getRightTriggerAxis() > 0.2 || OI.getDriverRB()) {
      // activate slowy spin
      turnLimit = 0.1;
      oiRX = oiRX * 0.8;
      oiLX = oiLX * 0.8;
      oiRY = oiRY * 0.8;
      oiLY = oiLY * 0.8;
    }
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    double turn = turnLimit
        * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.05) {
      turn = 0.0;
    }
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.drive(controllerVector, turn);
  }

  public void robotCentricDrive(double angle) {
    double oiRX = OI.getDriverRightX();
    double oiLX = OI.getDriverLeftX();
    double oiRY = OI.getDriverRightY();
    double oiLY = OI.getDriverLeftY();
    double speedMultiplier = (((60 - Constants.metersToInches(elevator.getElevatorPosition())) * 0.4 / 50) + 0.6);

    if (elevator.getElevatorPosition() > Constants.inchesToMeters(10)) {
      oiRX = oiRX * speedMultiplier;
      oiLX = oiLX * speedMultiplier;
      oiRY = oiRY * speedMultiplier;
      oiLY = oiLY * speedMultiplier;
    }

    // Logger.recordOutput("Adjusted Right X", oiRX);
    // Logger.recordOutput("Adjusted Left X", oiLX);
    // Logger.recordOutput("Adjusted Right Y", oiRY);
    // Logger.recordOutput("Adjusted Left Y", oiLY);

    double turnLimit = 0.17;
    // 0.35 before

    if (OI.driverController.getRightTriggerAxis() > 0.2) {
      // activate slowy spin
      turnLimit = 0.1;
      oiRX = oiRX * 0.5;
      oiLX = oiLX * 0.5;
      oiRY = oiRY * 0.5;
      oiLY = oiLY * 0.5;
    }

    // this is correct, X is forward in field, so originalX should be the y on the
    // // joystick
    // double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(),
    // OI.getDriverLeftY()));
    // double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(),
    // OI.getDriverLeftX()));
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    // if (Math.abs(originalX) < 0.005) {
    // originalX = 0;
    // }
    // if (Math.abs(originalY) < 0.005) {
    // originalY = 0;
    // }

    // double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() *
    // OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) *
    // (Constants.Physical.TOP_SPEED)/(Constants.Physical.ROBOT_RADIUS));
    double turn = turnLimit
        * (oiRX * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.05) {
      turn = 0.0;
    }

    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }
    io.driveCamCentric(controllerVector, turn, Math.toRadians(angle));
  }

  public void teleopDriveToPiece(double yToPiece) {
    double turnLimit = 0.17;
    double kP = 0.8;
    // 0.35 before

    // if (OI.driverController.getLeftBumper()) {
    // // activate speedy spin
    // // turnLimit = 1; //TODO: find a different keybind for this
    // }

    // this is correct, X is forward in field, so originalX should be the y on the
    // joystick
    double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
    double originalY = yToPiece * kP;

    if (Math.abs(originalX) < 0.075) {
      originalX = 0;
    }

    double turn = turnLimit
        * (OI.getDriverRightX() * (Constants.Physical.TOP_SPEED) / (Constants.Physical.ROBOT_RADIUS));

    if (Math.abs(turn) < 0.15) {
      turn = 0.0;
    }

    if (turn == 0.0) {
      double yaw = io.getYaw().getDegrees();

      double result = -2 * turningPID.updatePID(yaw);
      // Logger.recordOutput("result", result);

      double x = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
      double y = yToPiece * kP;

      if (Math.abs(originalX) < 0.05) {
        originalX = 0;
      }

      double pigeonAngle = io.getYaw().getRadians();
      double xPower = getAdjustedX(x, y);
      double yPower = getAdjustedY(x, y);

      double xSpeed = xPower * Constants.Physical.TOP_SPEED;
      double ySpeed = yPower * Constants.Physical.TOP_SPEED;

      Vector controllerVector = new Vector(xSpeed, ySpeed);
      if (getFieldSide().equals("red")) {
        controllerVector.setI(-xSpeed);
        controllerVector.setJ(-ySpeed);
      }
      io.drive(controllerVector, result);
    } else {
      double pigeonAngle = io.getYaw().getRadians();
      double xPower = getAdjustedX(originalX, originalY);
      double yPower = getAdjustedY(originalX, originalY);

      double xSpeed = xPower * Constants.Physical.TOP_SPEED;
      double ySpeed = yPower * Constants.Physical.TOP_SPEED;

      Vector controllerVector = new Vector(xSpeed, ySpeed);
      if (getFieldSide().equals("red")) {
        controllerVector.setI(-xSpeed);
        controllerVector.setJ(-ySpeed);
      }
      io.drive(controllerVector, turn);
    }
  }

  private int hitNumber = 0;
  private int hitNumberSemiGenerous = 0;
  private int hitNumberGenerous = 0;
  private int hitNumberUltraGenerous = 0;

  public boolean hitSetPoint(Pose2d pose) { // adjust for l4 TODO:
    // Logger.recordOutput("Error for setpoint",
    // Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    // System.out.println("X Y error: "
    // + Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2))
    // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
    // Math.toDegrees(getMt2Pose2dAngle()))
    // + " Hits: "
    // + hitNumber);
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    if (Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.045
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 1.5) {
      hitNumber += 1;
    } else {
      hitNumber = 0;
    }
    if (hitNumber > 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hitSetPointSemiGenerous(Pose2d pose) { // adjust for l4 TODO:
    // Logger.recordOutput("Error for setpoint",
    // Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    // System.out.println("X Y error: "
    // + Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2))
    // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
    // Math.toDegrees(getMt2Pose2dAngle()))
    // + " Hits: "
    // + hitNumber);
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    Logger.recordOutput("Error for semi-generous", Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)));
    if (Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.05
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 2) {
      hitNumberSemiGenerous += 1;
    } else {
      hitNumberSemiGenerous = 0;
    }
    if (hitNumberSemiGenerous > 3) {
      return true;
    } else {
      return false;
    }
  }

  public Pose2d getReefClosestSetpointFrontOnly(Pose2d currentOdometry /* {x, y, thetaRadians} */) {
    double x = currentOdometry.getX();
    double y = currentOdometry.getY();
    double theta = Constants.standardizeAngleDegrees(currentOdometry.getRotation().getDegrees());
    double dist = 100.0;
    double currentDist = 100.0;
    Pose2d chosenSetpoint = new Pose2d(x, y, new Rotation2d(Math.toRadians(theta)));
    if (getFieldSide() == "red") {
      for (int i = 0; i < Constants.Reef.redFrontPlacingPositions.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            x - (Constants.Reef.redFrontPlacingPositions.get(i).getX() + Constants.Reef.redBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.redFrontPlacingPositions.get(i).getY()
                + Constants.Reef.redBackPlacingPositions.get(i)
                    .getY())
                / 2);
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint = Constants.Reef.redFrontPlacingPositions.get(i);
        }
      }
    } else {
      for (int i = 0; i < Constants.Reef.blueFrontPlacingPositions.size(); i++) {
        currentDist = Math.hypot(
            x - (Constants.Reef.blueFrontPlacingPositions.get(i).getX() + Constants.Reef.blueBackPlacingPositions
                .get(i)
                .getX()) / 2,
            y - (Constants.Reef.blueFrontPlacingPositions.get(i).getY() + Constants.Reef.blueBackPlacingPositions.get(i)
                .getY()) / 2);
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint = Constants.Reef.blueFrontPlacingPositions.get(i);
        }
      }
    }
    if (chosenSetpoint.getTranslation().getDistance(currentOdometry.getTranslation()) > 5) {
      return getMt2Pose2d();
    } else {
      return chosenSetpoint;
    }
  }

  public boolean hitSetPointGenerous(Pose2d pose) { // adjust for l4 TODO:
    // Logger.recordOutput("Error for setpoint",
    // Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    // System.out.println("X Y error: "
    // + Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2))
    // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
    // Math.toDegrees(getMt2Pose2dAngle()))
    // + " Hits: "
    // + hitNumber);
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    if (Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.10
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 2.5) {
      hitNumberGenerous += 1;
    } else {
      hitNumberGenerous = 0;
    }
    if (hitNumberGenerous > 3) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hitSetPointUltraGenerous(Pose2d pose) { // adjust for l4 TODO:
    // Logger.recordOutput("Error for setpoint",
    // Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    // System.out.println("X Y error: "
    // + Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2))
    // + " Angle error: " + getAngleDifferenceDegrees(Math.toDegrees(theta),
    // Math.toDegrees(getMt2Pose2dAngle()))
    // + " Hits: "
    // + hitNumber);
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    if (Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.10
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 10.0) {
      hitNumberUltraGenerous += 1;
    } else {
      hitNumberUltraGenerous = 0;
    }
    if (hitNumberUltraGenerous > 2) {
      return true;
    } else {
      return false;
    }
  }

  public void driveToPoint(Pose2d targetPoint) {
    Logger.recordOutput("Goal X, Y, Theta", targetPoint);
    // Logger.recordOutput("Magnitude Error Inches",
    // Constants.metersToInches(Math.sqrt(Math.pow(x - getMt2Pose2dX(), 2) +
    // Math.pow(y - getMt2Pose2dY(), 2))));
    // Logger.recordOutput("Theta Error Degrees", Math.toDegrees(theta -
    // getMt2Pose2dAngle()));
    double x = targetPoint.getX();
    double y = targetPoint.getY();
    double theta = targetPoint.getRotation().getRadians();
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());

    double xVelNoFF = 0.0;
    double yVelNoFF = 0.0;
    double thetaVelNoFF = 0.0;

    if (DriverStation.isAutonomousEnabled() && systemState.equals(DriveState.L4_REEF)) {
      xxPID4A.setSetPoint(x);
      yyPID4A.setSetPoint(y);
      thetaaPID4A.setSetPoint(theta);

      xxPID4A.updatePID(getMt2Pose2dX());
      yyPID4A.updatePID(getMt2Pose2dY());
      thetaaPID4A.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID4A.getResult();
      yVelNoFF = yyPID4A.getResult();
      // double velmag = Math.hypot(xVelNoFF, yVelNoFF);
      // double maxvel = 0.1;
      // if (velmag > maxvel) {
      // xVelNoFF = xVelNoFF * maxvel / velmag;
      // yVelNoFF = yVelNoFF * maxvel / velmag;
      // }
      thetaVelNoFF = -thetaaPID4A.getResult();
    } else if (OI.driverPOVRight.getAsBoolean()) {
      xxPID4.setSetPoint(x);
      yyPID4.setSetPoint(y);
      thetaaPID4.setSetPoint(theta);

      xxPID4.updatePID(getMt2Pose2dX());
      yyPID4.updatePID(getMt2Pose2dY());
      thetaaPID4.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID4.getResult();
      yVelNoFF = yyPID4.getResult();
      thetaVelNoFF = -thetaaPID4.getResult();

    } else if (DriverStation.isTeleopEnabled()
        && (OI.driverPOVLeft.getAsBoolean() || OI.driverPOVDown.getAsBoolean())) {

      xxPID23.setSetPoint(x);
      yyPID23.setSetPoint(y);
      thetaaPID23.setSetPoint(theta);

      xxPID23.updatePID(getMt2Pose2dX());
      yyPID23.updatePID(getMt2Pose2dY());
      thetaaPID23.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID23.getResult();
      yVelNoFF = yyPID23.getResult();
      thetaVelNoFF = -thetaaPID23.getResult();

    } else if (DriverStation.isTeleopEnabled() && OI.driverPOVUp.getAsBoolean()) {

      xxPID1.setSetPoint(x);
      yyPID1.setSetPoint(y);
      thetaaPID1.setSetPoint(theta);

      xxPID1.updatePID(getMt2Pose2dX());
      yyPID1.updatePID(getMt2Pose2dY());
      thetaaPID1.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID1.getResult();
      yVelNoFF = yyPID1.getResult();
      thetaVelNoFF = -thetaaPID1.getResult();

    } else if (systemState.equals(DriveState.PIECE_PICKUP)) {

      xxPIDPickup.setSetPoint(x);
      yyPIDPickup.setSetPoint(y);
      thetaaPIDPickup.setSetPoint(theta);

      xxPIDPickup.updatePID(getMt2Pose2dX());
      yyPIDPickup.updatePID(getMt2Pose2dY());
      thetaaPIDPickup.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPIDPickup.getResult();
      yVelNoFF = yyPIDPickup.getResult();
      thetaVelNoFF = -thetaaPIDPickup.getResult();

    } else {

      xxPID.setSetPoint(x);
      yyPID.setSetPoint(y);
      thetaaPID.setSetPoint(theta);

      xxPID.updatePID(getMt2Pose2dX());
      yyPID.updatePID(getMt2Pose2dY());
      thetaaPID.updatePID(getMt2Pose2dAngle());

      xVelNoFF = xxPID.getResult();
      yVelNoFF = yyPID.getResult();
      thetaVelNoFF = -thetaaPID.getResult();
    }

    // double feedForwardX = targetPoint.getDouble("x_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardY = targetPoint.getDouble("y_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;
    // double feedForwardTheta = -targetPoint.getDouble("angular_velocity") *
    // Constants.Autonomous.FEED_FORWARD_MULTIPLIER;

    double finalX = xVelNoFF;
    double finalY = yVelNoFF;
    double finalTheta = thetaVelNoFF;
    // if (m_fieldSide == "blue") {
    // finalX = -finalX;
    // finalTheta = -finalTheta;
    // }
    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
    };

    Vector velocityVector = new Vector();
    double desiredThetaChange = 0;
    velocityVector.setI(velocityArray[0].doubleValue());
    velocityVector.setJ(velocityArray[1].doubleValue());
    desiredThetaChange = velocityArray[2].doubleValue();

    autoDrive(velocityVector, desiredThetaChange);

  }

  public void orbitDrive(Pose2d end, Pose2d pre, Pose2d current) {
    // end: C, pre: B, current: A
    Translation2d p = end.minus(pre).getTranslation(); // a vector in direction BC
    Translation2d d = pre.minus(current).getTranslation(); // a vector in direction AB
    Rotation2d alpha = p.getAngle().minus(d.getAngle()); // find angle between BC and AB
    Rotation2d direction = d.getAngle().minus(alpha);
    double velocity = getSpeedUsingPhysics(end.minus(current).getTranslation().getNorm(), 0);
    double xVel = velocity * Math.cos(direction.getRadians());
    double yVel = velocity * Math.sin(direction.getRadians());
    Vector velocityVector = new Vector(xVel, yVel);
    thetaaPID4.setSetPoint(end.getRotation().getRadians());
    double desiredThetaChange = thetaaPID4.getResult(); // TODO: Test if this PID is good.
    autoDrive(velocityVector, desiredThetaChange);
  }

  public double getSpeedUsingPhysics(double distance, double finalVel) {
    // Max Velocity at which the robot can slow down given the Max Deceleration
    // (-Constants.Physical.MAX_ACCELERATION)
    return Math.sqrt(finalVel * finalVel - (2 * (-Constants.Physical.MAX_ACCELERATION) * distance));
  }

  public double clampToForwardAccelerationLimit(double currentVelocity, double wantedAcceleration) {
    return Math.min(wantedAcceleration,
        Constants.Physical.MAX_ACCELERATION * (1 - (currentVelocity / Constants.Physical.TOP_SPEED)));
  }

  public void driveToXTheta(double x, double theta) {
    java.util.logging.Logger.getGlobal().finer(theta + "");
    // theta = Math.toRadians(theta);
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());
    xxPID.setSetPoint(x);
    thetaaPID.setSetPoint(theta);

    xxPID.updatePID(getMt2Pose2dX());
    thetaaPID.updatePID(getMt2Pose2dAngle());

    double xVelNoFF = xxPID.getResult();
    double yVelNoFF = OI.getDriverLeftX() * 2.9;
    double thetaVelNoFF = -thetaaPID.getResult();
    double finalX = xVelNoFF;
    double finalY = yVelNoFF;
    double finalTheta = thetaVelNoFF;
    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
    };

    Vector velocityVector = new Vector();
    double desiredThetaChange = 0;
    if (getFieldSide().equals("red")) {
      velocityVector.setI(velocityArray[0].doubleValue());
      velocityVector.setJ(-velocityArray[1].doubleValue());
    } else {
      velocityVector.setI(velocityArray[0].doubleValue());
      velocityVector.setJ(velocityArray[1].doubleValue());
    }
    desiredThetaChange = velocityArray[2].doubleValue();

    autoDrive(velocityVector, desiredThetaChange);
  }

  public void driveOnLine(Vector lineVector, Translation2d pointOnLine, double angrad) {
    Logger.recordOutput("Point On Line", new Pose2d(pointOnLine, new Rotation2d()));
    Logger.recordOutput("LineVector",
        new Pose2d(pointOnLine.plus(new Translation2d(lineVector.getI(), lineVector.getJ())), new Rotation2d()));
    double oiLY = OI.getDriverLeftY();
    double oiLX = OI.getDriverLeftX();
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    if (getFieldSide().equals("red")) {
      controllerVector.setI(-ySpeed);
      controllerVector.setJ(xSpeed);
    }
    Vector projectedJoystick = lineVector.projectOther(controllerVector);
    Logger.recordOutput("controllerProjection",
        new Pose2d(new Translation2d(projectedJoystick.getI() + getMt2Pose2dX(),
            projectedJoystick.getJ() + getMt2Pose2dY()), new Rotation2d(angrad)));
    Translation2d currentPoint = new Translation2d(getMt2Pose2dX(), getMt2Pose2dY());
    Translation2d closestPointOnLine = lineVector.getClosestPointOnLine(pointOnLine, currentPoint);
    Logger.recordOutput("closestPointOnLine", new Pose2d(closestPointOnLine, new Rotation2d(angrad)));
    xxPID.setSetPoint(closestPointOnLine.getX());
    yyPID.setSetPoint(closestPointOnLine.getY());
    angrad = Constants.standardizeAngleToOther(angrad, getMt2Pose2dAngle());
    thetaaPID.setSetPoint(angrad);
    double toPointXVel = xxPID.updatePID(getMt2Pose2dX());
    double toPointYVel = -yyPID.updatePID(getMt2Pose2dY());
    double thetaVel = -thetaaPID.updatePID(getMt2Pose2dAngle());
    Logger.recordOutput("feederAngle", Math.toDegrees(angrad));
    Logger.recordOutput("thetaVel", thetaVel);
    Logger.recordOutput("toPointYVel", toPointYVel);
    Logger.recordOutput("toPointXVel", toPointXVel);
    Vector toPointVector = new Vector(toPointXVel, toPointYVel);
    Vector driveVector = toPointVector.add(projectedJoystick);
    Logger.recordOutput("driveVector", new Pose2d(new Translation2d(driveVector.getI() + getMt2Pose2dX(),
        driveVector.getJ() + getMt2Pose2dY()), new Rotation2d(angrad)));
    autoDrive(driveVector, thetaVel);
  }

  public void driveToTheta(double theta) {
    theta = Constants.standardizeAngleToOtherDegrees(theta, getMt2Pose2dAngle());

    // Logger.recordOutput("Drive Angle Setpoint", theta);
    turningPID.setSetPoint(theta);
    turningPID.updatePID(Math.toDegrees(getMt2Pose2dAngle()));

    double result = -turningPID.getResult();
    if (Math.abs(Math.toDegrees(getMt2Pose2dAngle()) - theta) < 2) {
      result = 0;
    }
    driveAutoAligned(result);
  }

  /**
   * Runs autonomous driving by providing velocity vector and turning rate.
   * 
   * @param vector            The velocity vector containing xy velocities.
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoDrive(Vector vector, double turnRadiansPerSec) {
    io.drive(vector, turnRadiansPerSec);
  }

  /**
   * Retrieves the current velocity vector of the robot in field coordinates.
   * The velocity vector is calculated based on the individual wheel speeds and
   * orientations.
   *
   * @return The current velocity vector of the robot in meters per second (m/s).
   */
  public Vector getRobotVelocityVector() {
    Vector velocityVector = io.getVelocityVector();
    return velocityVector;
  }

  /**
   * Retrieves the path point closest to the specified time from the given path.
   * If the specified time is before the first path point, the first point is
   * returned.
   * If the specified time is after the last path point, the last point is
   * returned.
   *
   * @param path The array containing path points, each represented as a
   *             JSONArray.
   * @param time The time for which the closest path point is required.
   * @return The closest path point to the specified time.
   */
  public JSONArray getPathPoint(JSONArray path, double time) {
    for (int i = 0; i < path.length() - 1; i++) {
      JSONArray currentPoint = path.getJSONArray(i + 1);
      JSONArray previousPoint = path.getJSONArray(i);
      double currentPointTime = currentPoint.getDouble(0);
      double previousPointTime = previousPoint.getDouble(0);
      if (time >= previousPointTime && time < currentPointTime) {
        return currentPoint;
      }
    }
    if (time < path.getJSONArray(0).getDouble(0)) {
      return path.getJSONArray(0);
    } else {
      return path.getJSONArray(path.length() - 1);
    }
  }

  public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
      JSONArray pathPoints, boolean fullSend, boolean accurate) {
    JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
    int targetIndex = pathPoints.length() - 1;
    if (this.m_fieldSide == "blue") {
      currentX = Constants.Physical.FIELD_LENGTH - currentX;
      currentY = Constants.Physical.FIELD_WIDTH - currentY;
      currentTheta = Math.PI + currentTheta;
    }

    if (OI.isProcessorSide()) {
      currentY = Constants.Physical.FIELD_WIDTH - currentY;
      currentTheta = -currentTheta;
    }

    for (int i = currentIndex; i < pathPoints.length(); i++) {
      JSONObject point = pathPoints.getJSONObject(i);
      double targetX = point.getDouble("x"), targetY = point.getDouble("y"),
          targetTheta = point.getDouble("angle"), targetXvel = point.getDouble("x_velocity"),
          targetYvel = point.getDouble("y_velocity"), targetThetavel = point.getDouble("angular_velocity");
      targetTheta = Constants.standardizeAngleToOther(targetTheta, currentTheta);
      double linearVelMag = Math.hypot(targetYvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          targetXvel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
      double targetVelMag = Math.hypot(linearVelMag,
          targetThetavel / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
      double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
          : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
              + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;// If full send mode is enabled, use the full send lookahead
      double deltaX = (currentX - targetX), deltaY = (currentY - targetY), deltaTheta = (currentTheta - targetTheta);
      if (!insideRadius(deltaX / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          deltaY / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
          deltaTheta / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
          lookaheadRadius)) {
        targetIndex = i;
        targetPoint = pathPoints.getJSONObject(i);
        break;
      }
    }
    double targetX = targetPoint.getDouble("x"), targetY = targetPoint.getDouble("y"),
        targetTheta = targetPoint.getDouble("angle");

    targetTheta = Constants.standardizeAngleToOther(targetTheta, currentTheta);

    xPID.setSetPoint(targetX);
    yPID.setSetPoint(targetY);
    thetaPID.setSetPoint(targetTheta);

    xPID.updatePID(currentX);
    yPID.updatePID(currentY);
    thetaPID.updatePID(currentTheta);
    double pidScaler = 1;
    double xVelNoFF = xPID.getResult() * pidScaler;
    double yVelNoFF = yPID.getResult() * pidScaler;
    double thetaVelNoFF = -thetaPID.getResult();
    double f = (accurate ? Constants.Autonomous.ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY
        : Constants.Autonomous.FEED_FORWARD_MULTIPLIER);
    double feedForwardX = targetPoint.getDouble("x_velocity") * f;
    double feedForwardY = targetPoint.getDouble("y_velocity") * f;
    double feedForwardTheta = -targetPoint.getDouble("angular_velocity") * f * 0.1;

    double finalX = xVelNoFF + feedForwardX;
    double finalY = yVelNoFF + feedForwardY;
    double finalTheta = (thetaVelNoFF + feedForwardTheta) * 1.25;
    if (m_fieldSide == "blue") {
      finalX = -finalX;
      finalY = -finalY;
    }

    if (OI.isProcessorSide()) {
      finalY = -finalY;
      finalTheta = -finalTheta;
    }

    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
        targetIndex,
    };
    double linearVelMag = Math.hypot(
        targetPoint.getDouble("x_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        targetPoint.getDouble("y_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
    double targetVelMag = Math.hypot(linearVelMag,
        targetPoint.getDouble("angular_velocity") / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
    double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
        : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
            + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;

    Logger.recordOutput("x-vel", xVelNoFF);
    Logger.recordOutput("y-vel", yVelNoFF);
    Logger.recordOutput("theta-vel", thetaVelNoFF);
    Logger.recordOutput("wanted-theta-vel",
        targetPoint.getDouble("angular_velocity"));
    Logger.recordOutput("FF-theta-vel", feedForwardTheta);
    Logger.recordOutput("FF-x-vel", feedForwardX);
    Logger.recordOutput("FF-y-vel", feedForwardY);
    Logger.recordOutput("current point idx", currentIndex);
    Logger.recordOutput("point idx", velocityArray[3].intValue());
    Logger.recordOutput("look-ahead", lookaheadRadius);
    Logger.recordOutput("target-point", new Pose2d(targetX, targetY, new Rotation2d(targetTheta)));
    Logger.recordOutput("Velocity Array",
        new double[] { finalX, -finalY, finalTheta });
    Logger.recordOutput("dx", targetX - currentX);
    Logger.recordOutput("dy", targetY - currentY);
    Logger.recordOutput("dtheta", targetTheta - currentTheta);
    return velocityArray;
  }

  public boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
    // Logger.recordOutput("deltax", deltaX);
    // Logger.recordOutput("deltay", deltaY);
    // Logger.recordOutput("deltaTheta", deltaTheta);
    Logger.recordOutput("Error", Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)));
    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
  }

  public void calculateAngleChange(double angle) {
    double pigeonAngleDegrees = io.getYaw().getDegrees();
    double targetAngle = 0;
    if (getFieldSide() == "red") {
      targetAngle = angle + 180;
    } else {
      targetAngle = angle;
    }

    if (DriverStation.isAutonomousEnabled() && getFieldSide() == "red") {
      pigeonAngleDegrees = 180 + pigeonAngleDegrees;
    }
    this.turningPID.setSetPoint(Constants.standardizeAngleDegrees(targetAngle));
    this.turningPID.updatePID(Constants.standardizeAngleDegrees(pigeonAngleDegrees));
    double turnResult = -turningPID.getResult();

    this.driveAutoAligned(turnResult);
  }

  private DriveState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return DriveState.DEFAULT;
      case IDLE:
        return DriveState.IDLE;
      case REEF:
        return DriveState.REEF;
      case REEF_MORE:
        return DriveState.REEF_MORE;
      case BACK:
        return DriveState.BACK;
      case L3_REEF:
        return DriveState.L3_REEF;
      case L4_REEF:
        return DriveState.L4_REEF;
      case ALGAE:
        return DriveState.ALGAE;
      case ALGAE_MORE:
        return DriveState.ALGAE_MORE;
      case ALGAE_MORE_MORE:
        return DriveState.ALGAE_MORE_MORE;
      case PROCESSOR:
        return DriveState.PROCESSOR;
      case PROCESSOR_MORE:
        return DriveState.PROCESSOR_MORE;
      case NET:
        return DriveState.NET;
      case NET_MORE:
        return DriveState.NET_MORE;
      case FEEDER:
        return DriveState.FEEDER;
      case SCORE_L23:
        return DriveState.SCORE_L23;
      case AUTO_FEEDER:
        return DriveState.AUTO_FEEDER;
      case FEEDER_ALIGN:
        return DriveState.FEEDER_ALIGN;
      case AUTO_L1:
        return DriveState.AUTO_L1;
      case AUTO_L1_MORE:
        return DriveState.AUTO_L1_MORE;
      case FEEDER_AUTO:
        return DriveState.FEEDER_AUTO;
      case PIECE_PICKUP:
        return DriveState.PIECE_PICKUP;
      case AUTO_CLIMB:
        return DriveState.AUTO_CLIMB;
      case STOP:
        return DriveState.STOP;
      default:
        return DriveState.IDLE;
    }
  }

  Vector scoreL23Vector = new Vector(2.5, 0);
  Pose2d l23Setpoint = new Pose2d();

  Vector pickupAlgaeFrontVector = new Vector(2.5, 0);
  Pose2d algaeSetpoint = new Pose2d();
  Vector pickupAlgaeBackVector = new Vector(-2.5, 0);

  public double getDistanceFromL23Setpoint() {
    return l23Setpoint.getTranslation().getDistance(getMt2Pose2d().getTranslation());
  }

  public double getDistanceFromAlgaeSetpoint() {
    return algaeSetpoint.getTranslation().getDistance(getMt2Pose2d().getTranslation());
  }

  public double getThetaToCenterReef() {
    double theta = 0.0;
    if (OI.isRedSide()) {
      theta = Math.atan2(Constants.Reef.centerRed.getY() - getMt2Pose2dY(),
          Constants.Reef.centerRed.getX() - getMt2Pose2dX());
    } else {
      theta = Math.atan2((Constants.Reef.centerBlue.getY() - getMt2Pose2dY()),
          (Constants.Reef.centerBlue.getX() - getMt2Pose2dX()));
    }
    if (getAngleDifferenceDegrees(Math.toDegrees(theta), Math.toDegrees(getMt2Pose2dAngle())) > 90.0) {
      theta -= Math.PI;
    }
    return theta;
  }

  public double[] getClosestPoint(double[] lineStart, double[] lineEnd) { // chatGPT ahh code
    // System.out.println("X1 - X2; " + (lineStart[0] - lineEnd[0]) + "Y1 - Y2" +
    // (lineStart[1] - lineEnd[1]));
    double x1 = lineStart[0];
    double y1 = lineStart[1];
    double x2 = lineEnd[0];
    double y2 = lineEnd[1];
    double px = getMt2Pose2dX();
    double py = getMt2Pose2dY();

    double dx = x2 - x1;
    double dy = y2 - y1;
    double lenSq = dx * dx + dy * dy;

    if (lenSq == 0) {
      return new double[] { x1, y1 };
    }

    double t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));

    double closestX = x1 + t * dx;
    double closestY = y1 + t * dy;

    return new double[] { closestX, closestY };
  }

  public double getThetaToPoint(double xMeters, double yMeters) {
    return Math.atan2(yMeters - getMt2Pose2dY(),
        xMeters - getMt2Pose2dX());
  }

  public double[] getClosestL1PointXY() { // returns the closest point on the trough for l1
    // double[] xy = { 0.0, 0.0 };
    // if (isL1Face()) {
    // xy = getClosestPoint(getClosestL1Points().get(0),
    // getClosestL1Points().get(1));
    // } else {
    // xy = getClosestPoint(getClosestL1Points().get(0),
    // getClosestL1Points().get(0));
    // }
    // return xy;
    // System.out.println("X1: " + getClosestL1Points().get(0)[0] + " Y1: " +
    // getClosestL1Points().get(0)[1] + "X2: "
    // + getClosestL1Points().get(1)[0] + " Y2: " + getClosestL1Points().get(1)[1]);
    double[] xy = getClosestPoint(getClosestL1Points().get(0), getClosestL1Points().get(1));
    return xy;
  }

  public double[] getClosestL1CornerPointXY() { // returns the closest point on the trough for l1
    // double[] xy = { 0.0, 0.0 };
    // if (isL1Face()) {
    // xy = getClosestPoint(getClosestL1Points().get(0),
    // getClosestL1Points().get(1));
    // } else {
    // xy = getClosestPoint(getClosestL1Points().get(0),
    // getClosestL1Points().get(0));
    // }
    // return xy;
    // System.out.println("X1: " + getClosestL1Points().get(0)[0] + " Y1: " +
    // getClosestL1Points().get(0)[1] + "X2: "
    // + getClosestL1Points().get(1)[0] + " Y2: " + getClosestL1Points().get(1)[1]);
    double[] xy = getClosestPoint(getClosestL1CornerPoints().get(0), getClosestL1CornerPoints().get(1));
    return xy;
  }

  public ArrayList<double[]> getClosestL1Points() {
    ArrayList<double[]> list = new ArrayList<>();
    double currentDist = 100.0;
    double dist = 100.0;
    double[] chosenSetpoint = { 0.0, 0.0 };
    double[] secondPoint = { 0.0, 0.0 };
    if (isOnBlueSide()) {
      for (int i = 0; i < Constants.Reef.l1BlueDrivePoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1BlueDrivePoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1BlueDrivePoints.get(i).getY()));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1BlueDrivePoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1BlueDrivePoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      // System.out.println("Closest Point: " + list.get(0)[0] + " " +
      // list.get(0)[1]);
      currentDist = 100.0;
      dist = 100.0;
      for (int i = 0; i < Constants.Reef.l1BlueDrivePoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1BlueDrivePoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1BlueDrivePoints.get(i).getY()));
        if (currentDist < dist && Math.hypot(Math.abs(Constants.Reef.l1BlueDrivePoints.get(i).getX() - list.get(0)[0]),
            Math.abs(Constants.Reef.l1BlueDrivePoints.get(i).getY() - list.get(0)[1])) > 0.1) {
          dist = currentDist;
          // System.out.println(" ");
          // System.out.println("new X2: " + Constants.Reef.l1BluePoints.get(i).getX() +
          // "new Y2: "
          // + Constants.Reef.l1BluePoints.get(i).getY());
          // System.out.println("old X2: " + list.get(0)[0] + "old Y2: "
          // + list.get(0)[1]);
          secondPoint[0] = Constants.Reef.l1BlueDrivePoints.get(i).getX();
          secondPoint[1] = Constants.Reef.l1BlueDrivePoints.get(i).getY();
        }
      }
      // System.out.println("->->->->->->->->->");
      list.add(secondPoint);
      return list;
    } else {
      for (int i = 0; i < Constants.Reef.l1RedDrivePoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1RedDrivePoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1RedDrivePoints.get(i).getY()));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1RedDrivePoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1RedDrivePoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      currentDist = 100.0;
      dist = 100.0;
      chosenSetpoint[0] = 0.0;
      chosenSetpoint[1] = 0.0;
      for (int i = 0; i < Constants.Reef.l1RedDrivePoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1RedDrivePoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1RedDrivePoints.get(i).getY()));
        if (currentDist < dist && Math.abs(Constants.Reef.l1RedDrivePoints.get(i).getX() - list.get(0)[0]) > 0.1
            && Math.abs(Constants.Reef.l1RedDrivePoints.get(i).getY() - list.get(0)[1]) > 0.1) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1RedDrivePoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1RedDrivePoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      return list;

    }
  }

  public ArrayList<double[]> getClosestL1CornerPoints() {
    ArrayList<double[]> list = new ArrayList<>();
    double currentDist = 100.0;
    double dist = 100.0;
    double[] chosenSetpoint = { 0.0, 0.0 };
    double[] secondPoint = { 0.0, 0.0 };
    if (isOnBlueSide()) {
      for (int i = 0; i < Constants.Reef.l1BlueCornerPoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1BlueCornerPoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1BlueCornerPoints.get(i).getY()));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1BlueCornerPoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1BlueCornerPoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      // System.out.println("Closest Point: " + list.get(0)[0] + " " +
      // list.get(0)[1]);
      currentDist = 100.0;
      dist = 100.0;
      for (int i = 0; i < Constants.Reef.l1BlueCornerPoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1BlueCornerPoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1BlueCornerPoints.get(i).getY()));
        if (currentDist < dist && Math.hypot(Math.abs(Constants.Reef.l1BlueCornerPoints.get(i).getX() - list.get(0)[0]),
            Math.abs(Constants.Reef.l1BlueCornerPoints.get(i).getY() - list.get(0)[1])) > 0.1) {
          dist = currentDist;
          // System.out.println(" ");
          // System.out.println("new X2: " +
          // Constants.Reef.l1BlueCornerPoints.get(i).getX() + "new Y2: "
          // + Constants.Reef.l1BlueCornerPoints.get(i).getY());
          // System.out.println("old X2: " + list.get(0)[0] + "old Y2: "
          // + list.get(0)[1]);
          secondPoint[0] = Constants.Reef.l1BlueCornerPoints.get(i).getX();
          secondPoint[1] = Constants.Reef.l1BlueCornerPoints.get(i).getY();
        }
      }
      // System.out.println("->->->->->->->->->");
      list.add(secondPoint);
      return list;
    } else {
      for (int i = 0; i < Constants.Reef.l1RedCornerPoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1RedCornerPoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1RedCornerPoints.get(i).getY()));
        if (currentDist < dist) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1RedCornerPoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1RedCornerPoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      currentDist = 100.0;
      dist = 100.0;
      chosenSetpoint[0] = 0.0;
      chosenSetpoint[1] = 0.0;
      for (int i = 0; i < Constants.Reef.l1RedCornerPoints.size(); i++) {
        // currentDist = Math.sqrt(Math.pow((x -
        // Constants.Reef.redFrontPlacingPositions.get(i).getX()), 2)
        // + Math.pow((y - Constants.Reef.redFrontPlacingPositions.get(i).getY()), 2));
        currentDist = Math.hypot(
            getMt2Pose2dX() - (Constants.Reef.l1RedCornerPoints.get(i).getX()),
            getMt2Pose2dY() - (Constants.Reef.l1RedCornerPoints.get(i).getY()));
        if (currentDist < dist && Math.abs(Constants.Reef.l1RedCornerPoints.get(i).getX() - list.get(0)[0]) > 0.1
            && Math.abs(Constants.Reef.l1RedCornerPoints.get(i).getY() - list.get(0)[1]) > 0.1) {
          dist = currentDist;
          chosenSetpoint[0] = Constants.Reef.l1RedCornerPoints.get(i).getX();
          chosenSetpoint[1] = Constants.Reef.l1RedCornerPoints.get(i).getY();
        }
      }
      list.add(chosenSetpoint);
      return list;

    }
  }

  public boolean isL1Face() { // returns true if the robot is by a face of the L1, returns false if the robot
                              // is around the point on the edge of l1 faces
    if (isOnBlueSide()) {

    } else {

    }
    return true; // TODO: theoretically this should work because the closest
  }

  public Pose2d getClosestPose(Pose2d pose1, Pose2d pose2) {
    double dist1 = Math.hypot(Math.abs(pose1.getX() - getMt2Pose2dX()), Math.abs(pose1.getY() - getMt2Pose2dY()));
    double dist2 = Math.hypot(Math.abs(pose2.getX() - getMt2Pose2dX()), Math.abs(pose2.getY() - getMt2Pose2dY()));
    if (dist1 <= dist2) {
      return pose1;
    } else {
      return pose2;
    }
  }

  public Pose2d origionalSetpointPose = new Pose2d();
  public boolean firstTimeReef = true;

  public double getNetXSetpoint() {
    if (isOnBlueSide()) {
      return Constants.Reef.netBlueXM;
    } else {
      return Constants.Reef.netRedXM;
    }
  }

  public double getNetXMoreSetpoint() {
    if (isOnBlueSide()) {
      return Constants.Reef.netBlueXMore;
    } else {
      return Constants.Reef.netRedXMore;
    }
  }

  public double getNetThetaSetpoint() {
    if (isOnBlueSide()) {
      if (autoPlacingFront) {
        return Constants.Reef.netBlueFrontThetaR;
      } else {
        return Constants.Reef.netBlueBackThetaR;
      }
    } else {
      if (autoPlacingFront) {
        return Constants.Reef.netRedFrontThetaR;
      } else {
        return Constants.Reef.netRedBackThetaR;
      }
    }
  }

  public boolean isOnBlueSide() {
    return io.getPosition().getX() < Constants.Physical.FIELD_LENGTH / 2.0;
  }

  public double[] getNetXTheta() {
    double goalX = getMt2Pose2dX();
    double goalTheta = getMt2Pose2dAngle();

    if (isOnBlueSide()) {
      goalX = Constants.Reef.netBlueXM;
      if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
          Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
        autoPlacingFront = true;
        goalTheta = Constants.Reef.netBlueFrontThetaR;
      } else {
        autoPlacingFront = false;
        goalTheta = Constants.Reef.netBlueBackThetaR;
      }
    } else {
      goalX = Constants.Reef.netRedXM;
      if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
          Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
        autoPlacingFront = true;
        goalTheta = Constants.Reef.netRedFrontThetaR;
      } else {
        autoPlacingFront = false;
        goalTheta = Constants.Reef.netRedBackThetaR;
      }
    }

    double[] xTheta = { goalX, goalTheta };
    return xTheta;
  }

  public double[] getNetMoreXTheta() {
    double goalX = getMt2Pose2dX();
    double goalTheta = getMt2Pose2dAngle();

    if (isOnBlueSide()) {
      goalX = Constants.Reef.netBlueXMore;
      if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
          Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
        autoPlacingFront = true;
        goalTheta = Constants.Reef.netBlueFrontThetaR;
      } else {
        autoPlacingFront = false;
        goalTheta = Constants.Reef.netBlueBackThetaR;
      }
    } else {
      goalX = Constants.Reef.netRedXMore;
      if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
          Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
        autoPlacingFront = true;
        goalTheta = Constants.Reef.netRedFrontThetaR;
      } else {
        autoPlacingFront = false;
        goalTheta = Constants.Reef.netRedBackThetaR;
      }
    }

    double[] xTheta = { goalX, goalTheta };
    return xTheta;
  }

  public double distanceFromCenterOfReef() {
    if (isOnBlueSide()) {
      return Math.hypot((getMt2Pose2dX() - Constants.Reef.centerBlue.getX()),
          ((getMt2Pose2dY() - Constants.Reef.centerBlue.getY())));
    } else {
      return Math.hypot((getMt2Pose2dX() - Constants.Reef.centerRed.getX()),
          ((getMt2Pose2dY() - Constants.Reef.centerRed.getY())));
    }
  }

  private final Vector backupVector = new Vector(-20.0, 0.0);
  private final Vector otherBackupVector = new Vector(20.0, 0.0);

  @Override
  public void periodic() {
    // Pose2d target = getGamePiecePosition();
    // System.out.println(Math.toDegrees(getThetaToCenterReef()));
    // Translation2d t1 = new Translation2d(getMt2Pose2dX(), getMt2Pose2dY());
    // Rotation2d r1 = new Rotation2d(getThetaToCenterReef());
    // Pose2d p1 = new Pose2d(t1, r1);
    // Logger.recordOutput("L1 Auto Angle", p1);
    l23Setpoint = getReefClosestSetpoint(getMt2Pose2d(), false);
    algaeSetpoint = getAlgaeClosestSetpoint(getMt2Pose2d());
    io.update(systemState);
    // Logger.recordOutput("Robot Odometry", getMt2Pose2d());
    // process inputs
    DriveState newState = handleStateTransition();
    Pose2d setpoint = new Pose2d();
    double standardizedAngle = Constants.standardizeAngleDegrees(Math.toDegrees(getMt2Pose2dAngle()));
    if (newState != systemState) {
      systemState = newState;
    }
    Logger.recordOutput("Drive State", systemState);
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = DriveState.DEFAULT;
    }

    if (!systemState.equals(DriveState.AUTO_CLIMB)) {
      firstClimb = false;
    }
    if (!systemState.equals(DriveState.PIECE_PICKUP)) {
      firstTimeAutoPickup = false;
      firstTimeCalculated = false;
      firstTimeGoingInCalculated = false;
      firstTimeGoingIn = false;
      hasTrack = false;
    }
    if (!OI.getDriverA()) {
      firstTimeReef = true;
    }
    switch (systemState) {
      case DEFAULT:
        if (OI.driverA.getAsBoolean() && !(OI.driverPOVDown.getAsBoolean() || OI.driverPOVLeft.getAsBoolean()
            || OI.driverPOVUp.getAsBoolean() || OI.driverPOVRight.getAsBoolean())) {
          robotCentricDrive(195.0);
        } else {
          teleopDrive();
        }
        break;
      case IDLE:
        break;
      case STOP:
        Vector velocityVector = new Vector();
        velocityVector.setI(0);
        velocityVector.setJ(0);
        double desiredThetaChange = 0.0;
        autoDrive(velocityVector, desiredThetaChange);
        break;
      case AUTO_L1:
        // driveToTheta(Math.toDegrees(getThetaToCenterReef()));
        // driveToTheta(Math.toDegrees(getThetaToPoint(getClosestL1PointXY()[0],
        // getClosestL1PointXY()[1])));
        // driveOnLine(new Vector(getClosestL1Points().get(0)[0] -
        // getClosestL1Points().get(
        // 1)[0], getClosestL1Points().get(0)[1]
        // - getClosestL1Points().get(
        // 1)[1]),
        // new Translation2d(getClosestL1Points().get(0)[0], getClosestL1Points().get(
        // 0)[1]),
        // getThetaToPoint(getClosestL1CornerPointXY()[0],
        // getClosestL1CornerPointXY()[1]));
        // System.out.println(Math.toDegrees(getThetaToCenterReef()));
        if (firstTimeReef) {
          firstTimeReef = false;
          origionalSetpointPose = getL1ReefClosestSetpoint(getMt2Pose2d());
        }
        setpoint = getL1ReefClosestSetpoint(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case AUTO_L1_MORE:
        // driveToTheta(Math.toDegrees(getThetaToCenterReef()));
        // driveToTheta(Math.toDegrees(getThetaToPoint(getClosestL1PointXY()[0],
        // getClosestL1PointXY()[1])));
        // driveOnLine(new Vector(getClosestL1Points().get(0)[0] -
        // getClosestL1Points().get(
        // 1)[0], getClosestL1Points().get(0)[1]
        // - getClosestL1Points().get(
        // 1)[1]),
        // new Translation2d(getClosestL1Points().get(0)[0], getClosestL1Points().get(
        // 0)[1]),
        // getThetaToPoint(getClosestL1CornerPointXY()[0],
        // getClosestL1CornerPointXY()[1]));
        // System.out.println(Math.toDegrees(getThetaToCenterReef()));
        // if (firstTimeReef) {
        // firstTimeReef = false;
        // origionalSetpointPose = getL1ReefClosestSetpoint(getMt2Pose2d(), false);
        // }
        setpoint = getL1ReefClosestSetpointMore(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case REEF:
        if (firstTimeReef) {
          firstTimeReef = false;
          origionalSetpointPose = getReefClosestSetpoint(getMt2Pose2d(), false);
        }
        setpoint = getReefClosestSetpoint(getMt2Pose2d(), OI.getDriverA());
        driveToPoint(setpoint);
        break;
      case REEF_MORE:
        setpoint = getReefMoreClosestSetpoint(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case BACK:
        if (autoPlacingFront) {
          autoRobotCentricDrive(backupVector, 0.0);
        } else {
          autoRobotCentricDrive(otherBackupVector, 0.0);
        }
        break;
      case L4_REEF:
        if (firstTimeReef) {
          firstTimeReef = false;
          origionalSetpointPose = getReefL4ClosestSetpoint(getMt2Pose2d(), false);
        }
        // System.out.println(origionalSetpointPose);
        // Logger.recordOutput("Targeted L4 Point",
        // getReefL4ClosestSetpoint(getMt2Pose2d(), OI.getDriverA()));
        // TODO: make this work
        setpoint = getReefL4ClosestSetpoint(getMt2Pose2d(), OI.getDriverA());
        driveToPoint(setpoint);
        break;
      case L3_REEF:
        if (firstTimeReef) {
          firstTimeReef = false;
          origionalSetpointPose = getReefL3ClosestSetpoint(getMt2Pose2d(), false);
        }
        setpoint = getReefL3ClosestSetpoint(getMt2Pose2d(), OI.getDriverA());
        driveToPoint(setpoint);
        break;
      case PIECE_PICKUP:
        // Pose2d target = getGamePiecePosition();
        // if (!target.equals(new Pose2d())) {
        // targetPointPickup = target;
        // }
        // if (hasTrack) {
        goToCoral();
        java.util.logging.Logger.getGlobal().finer("Driving to point");
        // } else {
        // System.out.println("forward");
        // Vector v = new Vector();
        // v.setI(0.4);
        // v.setJ(0);
        // double d = 0.0;
        // autoRobotCentricDrive(v, d);
        // }
        break;
      case ALGAE:
        setpoint = getAlgaeClosestSetpoint(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case ALGAE_MORE:
        // if
        // (getAngleDifferenceDegrees(Math.toDegrees(getAlgaeClosestSetpoint(getMt2Pose2d())[2]),
        // Math.toDegrees(getMt2Pose2dAngle())) <= 90) {
        // autoRobotCentricDrive(scoreL23Vector, 0);
        // } else {
        // autoRobotCentricDrive(scoreL23Vector, 0);
        // }
        setpoint = getAlgaeMoreClosestSetpoint(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case ALGAE_MORE_MORE:
        setpoint = getAlgaeMoreMoreClosestSetpoint(getMt2Pose2d());
        driveToPoint(setpoint);
        break;
      case PROCESSOR:
        if (isOnBlueSide()) {
          if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
              Constants.Reef.processorBlueFrontPlacingPosition.getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorBlueFrontPlacingPosition);
            }
            // driveToTheta((Constants.Reef.processorBlueFrontPlacingPosition.getRotation().getDegrees()));
          } else {
            autoPlacingFront = false;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              // driveToTheta((Constants.Reef.processorBlueBackPlacingPosition.getRotation().getDegrees()));
              driveToPoint(Constants.Reef.processorBlueBackPlacingPosition);
            }
          }
        } else {
          if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
              Constants.Reef.processorRedFrontPlacingPosition.getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorRedFrontPlacingPosition);
            }
            // driveToTheta((Constants.Reef.processorRedFrontPlacingPosition.getRotation().getDegrees()));
          } else {
            autoPlacingFront = false;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorRedBackPlacingPosition);
            }
            // driveToTheta((Constants.Reef.processorRedBackPlacingPosition.getRotation().getDegrees()));
          }
        }
        break;
      case PROCESSOR_MORE:
        if (isOnBlueSide()) {
          if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
              Constants.Reef.processorMoreBlueFrontPlacingPosition.getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorMoreBlueFrontPlacingPosition);
            }
            // driveToTheta((Constants.Reef.processorMoreBlueFrontPlacingPosition.getRotation().getDegrees()));
          } else {
            autoPlacingFront = false;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              // driveToTheta((Constants.Reef.processorMoreBlueBackPlacingPosition.getRotation().getDegrees()));
              driveToPoint(Constants.Reef.processorMoreBlueBackPlacingPosition);
            }
          }
        } else {
          if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
              Constants.Reef.processorMoreRedFrontPlacingPosition.getRotation().getDegrees()) <= 90) {
            autoPlacingFront = true;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorMoreRedFrontPlacingPosition);
              // driveToTheta((Constants.Reef.processorMoreRedFrontPlacingPosition.getRotation().getDegrees()));
            }
          } else {
            autoPlacingFront = false;
            if (OI.driverA.getAsBoolean() || OI.driverLT.getAsBoolean()) {
              teleopDrive();
            } else {
              driveToPoint(Constants.Reef.processorMoreRedBackPlacingPosition);
            }
            // driveToTheta((Constants.Reef.processorMoreRedBackPlacingPosition.getRotation().getDegrees()));
          }
        }
        break;
      case NET_MORE:
        if (OI.getDriverA()) {
          teleopDrive();
        } else if (Math.abs(OI.getDriverLeftY()) > 0.2) {
          teleopDrive();
        } else {
          double[] setpointA = getNetMoreXTheta();
          driveToXTheta(setpointA[0], setpointA[1]);
        }

        // if (OI.isBlueSide()) { // TODO: uncomment to revert to colorado algae code
        // if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
        // Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
        // autoPlacingFront = true;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netBlueXMore,
        // Math.toDegrees(Constants.Reef.netBlueFrontThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netBlueFrontThetaR));
        // } else {
        // autoPlacingFront = false;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netBlueXMore,
        // Math.toDegrees(Constants.Reef.netBlueBackThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netBlueBackThetaR));
        // }
        // } else {
        // if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
        // Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
        // autoPlacingFront = true;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netRedXMore,
        // Math.toDegrees(Constants.Reef.netRedFrontThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netRedFrontThetaR));
        // } else {
        // autoPlacingFront = false;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netRedXMore,
        // Math.toDegrees(Constants.Reef.netRedBackThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netRedBackThetaR));
        // }
        // }

        break;
      case NET:
        if (OI.driverA.getAsBoolean()) {
          teleopDrive();
        } else {
          double[] setpointA = getNetXTheta();
          driveToXTheta(setpointA[0], setpointA[1]);
        }

        // if (OI.isBlueSide()) { // TODO: uncomment to revert to colorado algae code
        // if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
        // Math.toDegrees(Constants.Reef.netBlueFrontThetaR)) <= 90) {
        // autoPlacingFront = true;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netBlueXM,
        // Math.toDegrees(Constants.Reef.netBlueFrontThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netBlueFrontThetaR));
        // } else {
        // autoPlacingFront = false;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netBlueXM,
        // Math.toDegrees(Constants.Reef.netBlueBackThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netBlueBackThetaR));
        // }
        // } else {
        // if (getAngleDifferenceDegrees(Math.toDegrees(getMt2Pose2dAngle()),
        // Math.toDegrees(Constants.Reef.netRedFrontThetaR)) <= 90) {
        // autoPlacingFront = true;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netRedXM,
        // Math.toDegrees(Constants.Reef.netRedFrontThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netRedFrontThetaR));
        // } else {
        // autoPlacingFront = false;
        // if (OI.driverA.getAsBoolean()) {
        // teleopDrive();
        // } else {
        // driveToXTheta(Constants.Reef.netRedXM,
        // Math.toDegrees(Constants.Reef.netRedBackThetaR));
        // }
        // // driveToTheta(Math.toDegrees(Constants.Reef.netRedBackThetaR));
        // }
        // }

        break;
      case SCORE_L23:
        autoRobotCentricDrive(scoreL23Vector, 0);
        break;
      case FEEDER:
        if (getFieldSide() == "red") { // red side
          if (getMt2Pose2dY() > 4.026) { // redside right feeder (field top right)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              driveToTheta(234);
            } else { // robot back side redside left feeder (fieldside top right)
              driveToTheta(54);
            }
          } else { // redside left feeder (fieldside bottom right)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              driveToTheta(306);
            } else { // robot back side redside left (fieldside bottom right)
              driveToTheta(126);
            }
          }
        } else { // blue side
          if (getMt2Pose2dY() < 4.026) { // blue side right feeder (fieldside bottom left)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              driveToTheta(234);
            } else { // robot back side blueside right (fieldside bottom left)
              driveToTheta(54);
            }
          } else { // blue side left feeder (fieldside top left)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              driveToTheta(306);
            } else { // robot back side blueside left (fieldside top left)
              driveToTheta(126);
            }
          }
        }

        break;
      case AUTO_FEEDER:
        if (getFieldSide() == "red") { // red side
          if (getMt2Pose2dY() > 4.026) { // redside right feeder (field top right)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              // driveToTheta(234);
              driveToPoint(Constants.Reef.RED_RIGHT_FEEDER);
            } else { // robot back side redside left feeder (fieldside top right)
              // driveToTheta(54);
              driveToPoint(Constants.Reef.RED_RIGHT_FEEDER
                  .rotateAround(Constants.Reef.RED_RIGHT_FEEDER.getTranslation(), new Rotation2d(Math.PI)));
            }
          } else { // redside left feeder (fieldside bottom right)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              // driveToTheta(306);
              driveToPoint(Constants.Reef.RED_LEFT_FEEDER.rotateAround(Constants.Reef.RED_LEFT_FEEDER.getTranslation(),
                  new Rotation2d(Math.PI)));
            } else { // robot back side redside left (fieldside bottom right)
              // driveToTheta(126);
              driveToPoint(Constants.Reef.RED_LEFT_FEEDER);
            }
          }
        } else { // blue side
          if (getMt2Pose2dY() < 4.026) { // blue side right feeder (fieldside bottom left)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              // driveToTheta(234);
              driveToPoint(Constants.Reef.BLUE_RIGHT_FEEDER
                  .rotateAround(Constants.Reef.BLUE_RIGHT_FEEDER.getTranslation(), new Rotation2d(Math.PI)));
            } else { // robot back side blueside right (fieldside bottom left)
              // driveToTheta(54);
              driveToPoint(Constants.Reef.BLUE_RIGHT_FEEDER);
            }
          } else { // blue side left feeder (fieldside top left)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              // driveToTheta(306);
              driveToPoint(Constants.Reef.BLUE_LEFT_FEEDER);
            } else { // robot back side blueside left (fieldside top left)
              // driveToTheta(126);
              driveToPoint(Constants.Reef.BLUE_LEFT_FEEDER
                  .rotateAround(Constants.Reef.BLUE_LEFT_FEEDER.getTranslation(), new Rotation2d(Math.PI)));
            }
          }
        }
        break;
      case FEEDER_ALIGN:
        if (getFieldSide() == "red") { // red side
          if (getMt2Pose2dY() > 4.026) { // redside right feeder (field top right)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              Vector feederLine = new Vector(-Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getRadians());
              }
            } else { // robot back side redside left feeder (fieldside top right)
              Vector feederLine = new Vector(-Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.RED_RIGHT_FEEDER_TELEOP.getRotation().getRadians() + Math.PI);
              }
            }
          } else { // redside left feeder (fieldside bottom right)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              Vector feederLine = new Vector(-Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.RED_LEFT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getRadians() + Math.PI);
              }
            } else { // robot back side redside left feeder (fieldside top right)
              Vector feederLine = new Vector(-Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.RED_LEFT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.RED_LEFT_FEEDER_TELEOP.getRotation().getRadians());
              }
            }
          }
        } else { // blue side
          if (getMt2Pose2dY() < 4.026) { // blue side right feeder (fieldside bottom left)
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              Vector feederLine = new Vector(-Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getRadians() + Math.PI);
              }
            } else { // robot back side redside left feeder (fieldside top right)
              Vector feederLine = new Vector(-Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.BLUE_RIGHT_FEEDER_TELEOP.getRotation().getRadians());
              }
            }
          } else { // blue side left feeder (fieldside top left)
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              Vector feederLine = new Vector(-Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getRadians());
              }
            } else { // robot back side redside left feeder (fieldside top right)
              Vector feederLine = new Vector(-Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getSin(),
                  Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getCos());
              if (OI.driverA.getAsBoolean()) {
                teleopDrive();
              } else {
                driveOnLine(feederLine, Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getTranslation(),
                    Constants.Reef.BLUE_LEFT_FEEDER_TELEOP.getRotation().getRadians() + Math.PI);
              }
            }
          }
        }
        break;
      case FEEDER_AUTO:
        if (getFieldSide() == "red") { // red side
          if (getMt2Pose2dY() > 4.026) { // redside right feeder (field top right)
            Pose2d closestPose = getClosestPose(Constants.Reef.RED_RIGHT_FEEDER_LEFT,
                Constants.Reef.RED_RIGHT_FEEDER_RIGHT);
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              // driveToTheta(234);
              driveToPoint(closestPose);
            } else { // robot back side redside left feeder (fieldside top right)
              // driveToTheta(54);
              driveToPoint(closestPose.rotateAround(closestPose.getTranslation(), new Rotation2d(Math.PI)));
            }
          } else { // redside left feeder (fieldside bottom right)
            Pose2d closestPose = getClosestPose(Constants.Reef.RED_LEFT_FEEDER_LEFT,
                Constants.Reef.RED_LEFT_FEEDER_RIGHT);
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              // driveToTheta(306);
              driveToPoint(closestPose.rotateAround(closestPose.getTranslation(), new Rotation2d(Math.PI)));
            } else { // robot back side redside left (fieldside bottom right)
              // driveToTheta(126);
              driveToPoint(closestPose);
            }
          }
        } else { // blue side
          if (getMt2Pose2dY() < 4.026) { // blue side right feeder (fieldside bottom left)
            Pose2d closestPose = getClosestPose(Constants.Reef.BLUE_RIGHT_FEEDER_LEFT,
                Constants.Reef.BLUE_RIGHT_FEEDER_RIGHT);
            if ((standardizedAngle <= 324
                &&
                standardizedAngle >= 144)) {
              // driveToTheta(234);
              driveToPoint(closestPose.rotateAround(closestPose.getTranslation(),
                  new Rotation2d(Math.PI)));
            } else { // robot back side blueside right (fieldside bottom left)
              // driveToTheta(54);
              driveToPoint(closestPose);
            }
          } else { // blue side left feeder (fieldside top left)
            Pose2d closestPose = getClosestPose(Constants.Reef.BLUE_LEFT_FEEDER_LEFT,
                Constants.Reef.BLUE_LEFT_FEEDER_RIGHT);
            if ((standardizedAngle <= 36
                &&
                standardizedAngle >= 0)
                ||
                (standardizedAngle <= 360
                    &&
                    standardizedAngle >= 216)) {
              // driveToTheta(306);
              driveToPoint(
                  closestPose);
            } else { // robot back side blueside left (fieldside top left)
              // driveToTheta(126);
              driveToPoint(closestPose.rotateAround(closestPose.getTranslation(), new Rotation2d(Math.PI)));
            }
          }
        }

        break;
      case AUTO_CLIMB:
        Vector moveBack = new Vector(0.2, 0);

        if (!firstClimb) {
          startX = getMt2Pose2dX();
          startY = getMt2Pose2dY();
          firstClimb = true;
        }

        if (Math.hypot((Math.abs(getMt2Pose2dX() - startX)), Math.abs(getMt2Pose2dY() - startY)) < 0.15) {
          autoRobotCentricDrive(moveBack, 0);
        } else {
          autoRobotCentricDrive(new Vector(0, 0), 0);
        }
        break;

      default:

        break;
    }
  }

  /**
   * Calculates the adjusted y-coordinate based on the original x and y
   * coordinates.
   *
   * @param originalX The original x-coordinate.
   * @param originalY The original y-coordinate.
   * @return The adjusted y-coordinate.
   */
  public double getAdjustedY(double originalX, double originalY) {
    double adjustedY = originalY * Math.sqrt((1 - (Math.pow(originalX, 2)) / 2));
    return adjustedY;
  }

  /**
   * Calculates the adjusted x-coordinate based on the original x and y
   * coordinates.
   *
   * @param originalX The original x-coordinate.
   * @param originalY The original y-coordinate.
   * @return The adjusted x-coordinate.
   */
  public double getAdjustedX(double originalX, double originalY) {
    double adjustedX = originalX * Math.sqrt((1 - (Math.pow(originalY, 2)) / 2));
    return adjustedX;
  }
}