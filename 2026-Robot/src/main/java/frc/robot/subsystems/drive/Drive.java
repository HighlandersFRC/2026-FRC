package frc.robot.subsystems.drive;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {

  private DriveIO io;
  private Peripherals peripherals;

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

  public boolean algaeMode = false;

  public enum DriveState {
    DEFAULT,
    IDLE,
    STOP,
    SHOOTING,
  }

  private DriveState wantedState = DriveState.IDLE;
  private DriveState systemState = DriveState.IDLE;

  /**
   * Creates a new instance of the Swerve Drive subsystem.
   * Initializes the Swerve Drive subsystem with the provided peripherals.
   * 
   * @param peripherals The peripherals used by the Swerve Drive subsystem.
   */
  public Drive(Peripherals peripherals) {
    this.peripherals = peripherals;
    if (RobotBase.isReal()) {
      this.io = new DriveIOComp(this.peripherals);
    } else {
      this.io = new DriveIOSim();
    }
  }

  public void setWantedState(DriveState wantedState) {
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
  public void init() {
    // sets configurations when run on robot initalization
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
    io.setWheelsStraight();
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
    if (Globals.fieldSide == "blue") {
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

  }

  /**
   * Retrieves the current field side designation.
   * 
   * @return The current field side designation, indicating whether the robot is
   *         positioned on the "blue" or "red" side of the field.
   */
  public String getFieldSide() {
    return Globals.fieldSide;
  }

  public void setOdometry(Pose2d pose) {
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

  public double getAngleDifferenceDegrees(double angle1, double angle2) {
    double difference = Math.abs(angle1 - angle2) % 360;
    return difference > 180 ? 360 - difference : difference;
  }

  public double getGyroYaw() {
    return io.getYaw().getDegrees();
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

    double turnLimit = 0.17;

    if (OI.driverController.getRightTriggerAxis() > 0.2) {
      // activate slowy spin
      turnLimit = 0.1;
      oiRX = oiRX * 0.5;
      oiLX = oiLX * 0.5;
      oiRY = oiRY * 0.5;
      oiLY = oiLY * 0.5;
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
    io.driveCamCentric(controllerVector, turn, Math.toRadians(angle));
  }

  public void teleopDriveToPiece(double yToPiece) {
    double turnLimit = 0.17;
    double kP = 0.8;

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

      double x = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
      double y = yToPiece * kP;

      if (Math.abs(originalX) < 0.05) {
        originalX = 0;
      }

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

  public boolean hitSetPoint(Pose2d pose) {
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

  public boolean hitSetPointSemiGenerous(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    // Logger.recordOutput("Error for semi-generous", Math
    // .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
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

  public boolean hitSetPointGenerous(Pose2d pose) {
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

  public boolean hitSetPointUltraGenerous(Pose2d pose) {
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
    double x = targetPoint.getX();
    double y = targetPoint.getY();
    double theta = targetPoint.getRotation().getRadians();
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());

    double xVelNoFF = 0.0;
    double yVelNoFF = 0.0;
    double thetaVelNoFF = 0.0;

    if (OI.driverPOVRight.getAsBoolean()) {
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
    velocityVector.setI(velocityArray[0].doubleValue());
    velocityVector.setJ(velocityArray[1].doubleValue());
    desiredThetaChange = velocityArray[2].doubleValue();

    autoDrive(velocityVector, desiredThetaChange);

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
    Logger.recordOutput("Drive Angle Setpoint", theta);
    theta = Constants.standardizeAngleToOtherDegrees(theta, getMt2Pose2dAngle());

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
    if (Globals.fieldSide == "blue") {
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
    if (Globals.fieldSide == "blue") {
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
    Logger.recordOutput("Error inside radius",
        Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)));
    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
  }

  private DriveState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return DriveState.DEFAULT;
      case IDLE:
        return DriveState.IDLE;
      case STOP:
        return DriveState.STOP;
      case SHOOTING:
        return DriveState.SHOOTING;
      default:
        return DriveState.IDLE;
    }
  }

  public boolean isOnBlueSide() {
    return io.getPosition().getX() < Constants.Physical.FIELD_LENGTH / 2.0;
  }

  private double goalShootingTheta = 0.0;

  public void setGoalShootingTheta(double theta) {
    goalShootingTheta = theta;
  }

  public double getGoalShootingTheta() {
    return goalShootingTheta;
  }

  @Override
  public void periodic() {
    io.update(systemState);
    // process inputs
    DriveState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    Logger.recordOutput("Drive State", systemState);
    Logger.recordOutput("MT2 Odometry", getMt2Pose2d());
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = DriveState.DEFAULT;
    }
    switch (systemState) {
      case DEFAULT:
        // if (OI.driverA.getAsBoolean() && !(OI.driverPOVDown.getAsBoolean() ||
        // OI.driverPOVLeft.getAsBoolean()
        // || OI.driverPOVUp.getAsBoolean() || OI.driverPOVRight.getAsBoolean())) {
        // robotCentricDrive(195.0);
        // } else {
        teleopDrive();
        // }
        break;
      case IDLE:
        break;
      case SHOOTING:
        driveToTheta(getGoalShootingTheta());
        break;
      case STOP:
        Vector velocityVector = new Vector();
        velocityVector.setI(0);
        velocityVector.setJ(0);
        double desiredThetaChange = 0.0;
        autoDrive(velocityVector, desiredThetaChange);
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