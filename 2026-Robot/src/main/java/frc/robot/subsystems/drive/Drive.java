package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.RobotState;
import frc.robot.Constants.Field;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

// **Zero Wheels with the bolt head showing on the left when the front side(battery) is facing down/away from you**

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();

  private DriveIO io;
  private Peripherals peripherals;

  // path following PID values
  private double kXP = 4.00;
  private double kXI = 0.00;
  private double kXD = 1.20;

  private double kYP = kXP;
  private double kYI = kXI;
  private double kYD = kXD;

  private double kThetaP = 1.00;
  private double kThetaI = 0.00;
  private double kThetaD = 2.00;

  // auto climb
  private double kkXP = 2.5;
  private double kkXI = 0.00;
  private double kkXD = 0.60;

  private double kkYP = kkXP;
  private double kkYI = kkXI;
  private double kkYD = kkXD;

  private double kkThetaP = 1.79;
  private double kkThetaI = 0.00;
  private double kkThetaD = 0.971;

  // teleop targeting PID values
  private double kTurningP = 0.04;
  private double kTurningI = 0;
  private double kTurningD = 0.06;

  private PID xxPID = new PID(kkXP, kkXI, kkXD);
  private PID yyPID = new PID(kkYP, kkYI, kkYD);
  private PID thetaaPID = new PID(kkThetaP, kkThetaI, kkThetaD);

  private PID xPID = new PID(kXP, kXI, kXD);
  private PID yPID = new PID(kYP, kYI, kYD);
  private PID thetaPID = new PID(kThetaP, kThetaI, kThetaD);
  private PID turningPID = new PID(kTurningP, kTurningI, kTurningD);

  public boolean robotCentric = false;

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  private ChassisSpeeds acceleration = new ChassisSpeeds();

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Physical.Drive.xAccelLimit);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Physical.Drive.yAccelLimit);
  Debouncer xDebouncer = new Debouncer(Constants.Physical.Drive.xDebounceLimit);
  Debouncer yDebouncer = new Debouncer(Constants.Physical.Drive.yDebounceLimit);
  ChassisSpeeds previousControllerSpeeds = new ChassisSpeeds();

  public enum DriveState {
    DEFAULT,
    DEFAULT_SLOW,
    DEFAULT_SLOWISH,
    IDLE,
    IDLE_SLOW,
    STOP,
    DRIVE_TO_PRE_CLIMB,
    DRIVE_TO_ALIGN_CLIMB,
    DRIVE_TO_ALIGN_CLIMB_FINISH,
    SNAKE,
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
    xxPID.setMinOutput(-1.0);
    xxPID.setMaxOutput(1.0);

    yyPID.setMinOutput(-1.0);
    yyPID.setMaxOutput(1.0);

    thetaaPID.setMinOutput(-3.0);
    thetaaPID.setMaxOutput(3.0);

    xPID.setMinOutput(-4.9);
    xPID.setMaxOutput(4.9);

    yPID.setMinOutput(-4.9);
    yPID.setMaxOutput(4.9);

    thetaPID.setMinOutput(-3);
    thetaPID.setMaxOutput(3);

    turningPID.setMinOutput(-3.0);
    turningPID.setMaxOutput(3.0);

    xDebouncer.setDebounceType(DebounceType.kBoth);
    yDebouncer.setDebounceType(DebounceType.kBoth);
  }

  public void teleopInit() {
    io.setDriveCurrentLimits(Constants.Physical.Drive.NORMAL_DRIVE_CURRENT_LIMIT);
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

    if (OI.isLeftSide()) {
      firstPointY = Constants.Physical.FIELD_WIDTH - firstPointY;
      firstPointAngle = -firstPointAngle;
    }
    Pose2d firstPose2d = new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle));
    io.setDriveCurrentLimits(Constants.Physical.Drive.NORMAL_DRIVE_CURRENT_LIMIT);
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
    return RobotState.getInstance().getEstimatedPose();
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

    // if (OI.driverController.getRightTriggerAxis() > 0.2 || OI.getDriverRB()) {
    // // activate slowy spin
    // turnLimit = 0.1;
    // oiRX = oiRX * 0.8;
    // oiLX = oiLX * 0.8;
    // oiRY = oiRY * 0.8;
    // oiLY = oiLY * 0.8;
    // }
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
    ChassisSpeeds controllerSpeeds = new ChassisSpeeds(controllerVector.getI(), controllerVector.getJ(), turn);

    if (getFieldSide().equals("red")) {
      controllerVector.setI(-xSpeed);
      controllerVector.setJ(-ySpeed);
    }

    // boolean xDecreasing =
    // xDebouncer.calculate(Math.abs(currentSpeeds.vxMetersPerSecond) < Math
    // .abs(previousSpeeds.vxMetersPerSecond));
    // boolean yDecreasing =
    // yDebouncer.calculate(Math.abs(currentSpeeds.vyMetersPerSecond) < Math
    // .abs(previousSpeeds.vyMetersPerSecond));

    previousControllerSpeeds = controllerSpeeds;
    // double vx = xLimiter.calculate(controllerVector.getI());
    // double vy = yLimiter.calculate(controllerVector.getJ());
    if (wantedState == DriveState.DEFAULT_SLOW) {
      // if (!xDecreasing) {
      // controllerVector.setI(vx);
      // xLimiter.reset(vx);
      // }
      // if (!yDecreasing) {
      // controllerVector.setJ(vy);
      // yLimiter.reset(vy);
      // }
      controllerVector = controllerVector.scaled(0.41);
      controllerVector = controllerVector.cap(0.67);
      turn *= 0.41;
      if (Math.abs(turn) > Math.PI / 4.0) {
        turn = Math.PI / 4.0 * Math.copySign(1, turn);
      }
    }
    if (wantedState == DriveState.DEFAULT_SLOWISH) {
      // if (!xDecreasing) {
      // controllerVector.setI(vx);
      // xLimiter.reset(vx);
      // }
      // if (!yDecreasing) {
      // controllerVector.setJ(vy);
      // yLimiter.reset(vy);
      // }
      controllerVector = controllerVector.scaled(0.9);
      controllerVector = controllerVector.cap(0.8);
      // turn *= 0.67;
      if (Math.abs(turn) > Math.PI / 4.0) {
        turn = Math.PI / 4.0 * Math.copySign(1, turn);
      }
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

  private int hitNumber = 0;
  private int hitNumberSemiGenerous = 0;
  private int hitNumberGenerous = 0;
  private int hitNumberUltraGenerous = 0;
  private int hitNumberClimb = 0;

  public boolean hitSetPoint(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    if (Math
        .sqrt(Math.pow((x - getMt2Pose2dX()), 2)
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.01690
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 1.5) {
      hitNumber += 1;
    } else {
      hitNumber = 0;
    }
    // Logger.recordOutput("error", Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    if (hitNumber > 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hitSetPointClimb(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    if (Math.abs(x - getMt2Pose2dX()) < 0.02 && Math.abs(y - getMt2Pose2dY()) < 0.01690
        && getAngleDifferenceDegrees(Math.toDegrees(theta),
            Math.toDegrees(getMt2Pose2dAngle())) < 1.5) {
      hitNumberClimb += 1;
    } else {
      hitNumberClimb = 0;
    }
    // Logger.recordOutput("error", Math.sqrt(Math.pow((x - getMt2Pose2dX()), 2)
    // + Math.pow((y - getMt2Pose2dY()), 2)));
    if (hitNumberClimb > 3) {
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
            + Math.pow((y - getMt2Pose2dY()), 2)) < 0.02
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
    // Logger.recordOutput("Drive/Goal X, Y, Theta", targetPoint);
    double x = targetPoint.getX();
    double y = targetPoint.getY();
    double theta = targetPoint.getRotation().getRadians();
    theta = Constants.standardizeAngleToOther(theta, getMt2Pose2dAngle());

    double xVelNoFF = 0.0;
    double yVelNoFF = 0.0;
    double thetaVelNoFF = 0.0;
    xxPID.setSetPoint(x);
    yyPID.setSetPoint(y);
    thetaaPID.setSetPoint(theta);

    xxPID.updatePID(getMt2Pose2dX());
    yyPID.updatePID(getMt2Pose2dY());
    thetaaPID.updatePID(getMt2Pose2dAngle());

    xVelNoFF = xxPID.getResult();
    yVelNoFF = yyPID.getResult();
    thetaVelNoFF = -thetaaPID.getResult();
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

  public void stop() {
    io.drive(new Vector(), 0.0);
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
    // Logger.recordOutput("Drive/Point On Line", new Pose2d(pointOnLine, new
    // Rotation2d()));
    // Logger.recordOutput("Drive/LineVector",
    // new Pose2d(pointOnLine.plus(new Translation2d(lineVector.getI(),
    // lineVector.getJ())), new Rotation2d()));
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
    Translation2d currentPoint = new Translation2d(getMt2Pose2dX(), getMt2Pose2dY());
    Translation2d closestPointOnLine = lineVector.getClosestPointOnLine(pointOnLine, currentPoint);
    xxPID.setSetPoint(closestPointOnLine.getX());
    yyPID.setSetPoint(closestPointOnLine.getY());
    angrad = Constants.standardizeAngleToOther(angrad, getMt2Pose2dAngle());
    thetaaPID.setSetPoint(angrad);
    double toPointXVel = xxPID.updatePID(getMt2Pose2dX());
    double toPointYVel = -yyPID.updatePID(getMt2Pose2dY());
    double thetaVel = -thetaaPID.updatePID(getMt2Pose2dAngle());
    Vector toPointVector = new Vector(toPointXVel, toPointYVel);
    Vector driveVector = toPointVector.add(projectedJoystick);
    autoDrive(driveVector, thetaVel);
  }

  public void driveToTheta(double theta) {
    // Logger.recordOutput("Drive/Drive Angle Setpoint", theta);
    theta = Constants.standardizeAngleToOtherDegrees(theta, getMt2Pose2d().getRotation().getDegrees());

    turningPID.setSetPoint(theta);
    turningPID.updatePID(Math.toDegrees(getMt2Pose2dAngle()));

    double result = -turningPID.getResult();
    if (Math.abs(Math.toDegrees(getMt2Pose2dAngle()) - theta) < 2) {
      result = 0;
    }
    driveAutoAligned(result);
  }

  public void snakeDrive() {
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

    if (OI.driverController.getRightTriggerAxis() > 0.2) {
      // activate slowy spin
      oiRX = oiRX * 0.8;
      oiLX = oiLX * 0.8;
      oiRY = oiRY * 0.8;
      oiLY = oiLY * 0.8;
    }
    double originalX = -(Math.copySign(oiLY * oiLY, oiLY));
    double originalY = -(Math.copySign(oiLX * oiLX, oiLX));
    double xPower = getAdjustedX(originalX, originalY);
    double yPower = getAdjustedY(originalX, originalY);

    double xSpeed = xPower * Constants.Physical.TOP_SPEED;
    double ySpeed = yPower * Constants.Physical.TOP_SPEED;

    Vector controllerVector = new Vector(xSpeed, ySpeed);
    double theta = controllerVector.getRotation().getDegrees();
    if (getFieldSide().equals("red")) {
      driveToTheta(-theta + 180);
    } else {
      driveToTheta(-theta);
    }
  }

  /**
   * Runs autonomous driving by providing velocity vector and turning rate.
   * 
   * @param vector            The velocity vector containing xy velocities.
   * @param turnRadiansPerSec The rate at which the robot should spin in radians
   *                          per second.
   */
  public void autoDrive(Vector vector, double turnRadiansPerSec) {
    if (wantedState == DriveState.IDLE_SLOW) {
      vector = vector.scaled(0.41);
      vector = vector.cap(0.67);
      turnRadiansPerSec *= 0.41;
      if (Math.abs(turnRadiansPerSec) > Math.PI / 4.0) {
        turnRadiansPerSec = Math.PI / 4.0 * Math.copySign(1, turnRadiansPerSec);
      }
    }
    io.drive(vector, turnRadiansPerSec);
  }

  /**
   * Retrieves the current velocity vector of the robot in field coordinates.
   * The velocity vector is calculated based on the individual wheel speeds and
   * orientations.
   *
   * @return The current velocity vector of the robot in meters per second (m/s).
   */
  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds velocityVector = io.getChassisSpeeds();
    return velocityVector;
  }

  public ChassisSpeeds getPredictedDriveVelocityFromSim(double secondsInFuture) {
    ChassisSpeeds expected = Constants.Simulation.getExpectedDriveSpeeds(secondsInFuture,
        getChassisSpeeds(),
        io.getWantedChassisSpeeds());
    return expected;
  }

  /**
   * Retrieves the current angular velocity of the robot in radians/s.
   * The velocity vector is calculated based on the individual wheel speeds and
   * orientations.
   *
   * @return The current angular velocity of the robot in radians per second.
   */
  public double getRobotAngularVelocity() {
    double angVel = io.getChassisSpeeds().omegaRadiansPerSecond;
    return angVel;
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

    if (OI.isLeftSide()) {
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

    if (OI.isLeftSide()) {
      finalY = -finalY;
      finalTheta = -finalTheta;
    }

    if (Field.isNearBump(getMt2Pose2d().getTranslation())) { // if on the bump,
      // slow down to maintain control
      finalTheta = finalTheta * 0.75;
      finalX = finalX * 0.75;
      finalY = finalY * 0.75;
    }

    Number[] velocityArray = new Number[] {
        finalX,
        -finalY,
        finalTheta,
        targetIndex,
    };
    double linearVelMag = Math.hypot(
        targetPoint.getDouble("x_velocity") /
            Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        targetPoint.getDouble("y_velocity") /
            Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS);
    double targetVelMag = Math.hypot(linearVelMag,
        targetPoint.getDouble("angular_velocity") /
            Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS);
    double lookaheadRadius = fullSend ? Constants.Autonomous.FULL_SEND_LOOKAHEAD
        : Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_DISTANCE * targetVelMag
            + Constants.Autonomous.MIN_LOOKAHEAD_DISTANCE;

    Logger.recordOutput("Auto/Wanted Speed", Math.hypot(finalX, finalY));

    Logger.recordOutput("Auto/x-vel", xVelNoFF);
    Logger.recordOutput("Auto/y-vel", yVelNoFF);
    Logger.recordOutput("Auto/theta-vel", thetaVelNoFF);
    Logger.recordOutput("Auto/wanted-theta-vel",
        targetPoint.getDouble("angular_velocity"));
    Logger.recordOutput("Auto/FF-theta-vel", feedForwardTheta);
    Logger.recordOutput("Auto/FF-x-vel", feedForwardX);
    Logger.recordOutput("Auto/FF-y-vel", feedForwardY);
    Logger.recordOutput("Auto/current point idx", currentIndex);
    Logger.recordOutput("Auto/point idx", velocityArray[3].intValue());
    Logger.recordOutput("Auto/look-ahead", lookaheadRadius);
    Logger.recordOutput("Auto/target-point", new Pose2d(targetX, targetY, new Rotation2d(targetTheta)));
    Logger.recordOutput("Auto/Velocity Array",
        new double[] { finalX, -finalY, finalTheta });
    Logger.recordOutput("Auto/dx", targetX - currentX);
    Logger.recordOutput("Auto/dy", targetY - currentY);
    Logger.recordOutput("Auto/dtheta", targetTheta - currentTheta);
    return velocityArray;
  }

  public boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
    // Logger.recordOutput("Error inside radius",
    // Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta,
    // 2)));
    return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
  }

  private DriveState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return DriveState.DEFAULT;
      case DEFAULT_SLOW:
        return DriveState.DEFAULT_SLOW;
      case DEFAULT_SLOWISH:
        return DriveState.DEFAULT_SLOWISH;
      case IDLE:
        return DriveState.IDLE;
      case IDLE_SLOW:
        return DriveState.IDLE_SLOW;
      case STOP:
        return DriveState.STOP;
      case DRIVE_TO_ALIGN_CLIMB:
        return DriveState.DRIVE_TO_ALIGN_CLIMB;
      case DRIVE_TO_ALIGN_CLIMB_FINISH:
        return DriveState.DRIVE_TO_ALIGN_CLIMB_FINISH;
      case DRIVE_TO_PRE_CLIMB:
        return DriveState.DRIVE_TO_PRE_CLIMB;
      case SNAKE:
        return DriveState.SNAKE; // disable this for now
      default:
        return DriveState.IDLE;
    }
  }

  public boolean isOnBlueSide() {
    return getMt2Pose2d().getX() < Constants.Physical.FIELD_LENGTH / 2.0;
  }

  Field2d field = new Field2d();

  public Pose2d getClimbPrepSetpoint() {
    if (Globals.fieldSide.equals("blue")) {
      double distanceFromRightBlueSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.preClimbPoseRightBlueSide.getTranslation());
      double distanceFromLeftBlueSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.preClimbPoseLeftBlueSide.getTranslation());
      if (distanceFromRightBlueSide < distanceFromLeftBlueSide) {
        return Constants.Physical.preClimbPoseRightBlueSide;
      } else {
        return Constants.Physical.preClimbPoseLeftBlueSide;
      }
    } else {
      double distanceFromRightRedSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.preClimbPoseRightRedSide.getTranslation());
      double distanceFromLeftRedSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.preClimbPoseLeftRedSide.getTranslation());
      if (distanceFromRightRedSide < distanceFromLeftRedSide) {
        return Constants.Physical.preClimbPoseRightRedSide;
      } else {
        return Constants.Physical.preClimbPoseLeftRedSide;
      }
    }
  }

  public Pose2d getClimbAlignSetpoint() {
    if (Globals.fieldSide.equals("blue")) {
      double distanceFromRightBlueSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.climbPoseRightBlueSide.getTranslation());
      double distanceFromLeftBlueSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.climbPoseLeftBlueSide.getTranslation());
      if (distanceFromRightBlueSide < distanceFromLeftBlueSide) {
        return Constants.Physical.climbPoseRightBlueSide;
      } else {
        return Constants.Physical.climbPoseLeftBlueSide;
      }
    } else {
      double distanceFromRightRedSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.climbPoseRightRedSide.getTranslation());
      double distanceFromLeftRedSide = getMt2Pose2d().getTranslation()
          .getDistance(Constants.Physical.climbPoseLeftRedSide.getTranslation());
      if (distanceFromRightRedSide < distanceFromLeftRedSide) {
        return Constants.Physical.climbPoseRightRedSide;
      } else {
        return Constants.Physical.climbPoseLeftRedSide;
      }
    }
  }

  public ChassisSpeeds getFutureVelocity() {

    previousSpeeds = currentSpeeds;
    currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        getChassisSpeeds(),
        getMt2Pose2d().getRotation());

    acceleration = currentSpeeds.minus(previousSpeeds)
        .times(Globals.loopPeriodSecs == 0.0 ? 0.0 : 1.0 / Globals.loopPeriodSecs);
    // Logger.recordOutput("Drive/Acceleration", acceleration);
    ChassisSpeeds futureVelocity = currentSpeeds.plus(acceleration.times(Constants.Physical.Drive.velLookaheadTime));
    futureVelocity.omegaRadiansPerSecond = currentSpeeds.omegaRadiansPerSecond;
    return futureVelocity;
  }

  public boolean isFlat() {
    return io.getFlat();
  }

  public void lowerCurrentLimits() {
    io.setDriveCurrentLimits(Constants.Physical.Drive.LOW_DRIVE_CURRENT_LIMIT);
    io.setAngleCurrentLimits(Constants.Physical.Drive.LOW_TURN_CURRENT_LIMIT);
  }

  public void resetCurrentLimits() {
    io.setDriveCurrentLimits(Constants.Physical.Drive.NORMAL_DRIVE_CURRENT_LIMIT);
    io.setAngleCurrentLimits(Constants.Physical.Drive.NORMAL_TURN_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Field", field);
    field.setRobotPose(getMt2Pose2d());
    io.update(systemState);
    RobotState.getInstance().setRobotVelocity(getChassisSpeeds());
    RobotState.getInstance().setRobotSetpointVelocity(io.getWantedChassisSpeeds());

    // if (robotCentric) {
    // Logger.recordOutput("Drive/Driving Mode", "Robot Centric");
    // } else {
    // Logger.recordOutput("Drive/Driving Mode", "Field Centric");
    // }

    // process inputs
    DriveState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    Logger.recordOutput("States/Drive State", systemState);
    Logger.recordOutput("Drive/Drive State", systemState);
    Logger.recordOutput("Drive/MT2 Odometry", getMt2Pose2d());
    // Logger.recordOutput("Drive/Expected Speed",
    // Constants.chassisSpeedsToVector(getPredictedDriveVelocityFromSim(1.0)).magnitude());
    Logger.recordOutput("Drive/Actual Speed",
        Constants.chassisSpeedsToVector(getChassisSpeeds()).magnitude());
    Logger.recordOutput("Testing/Feed Setpoint",
        new Pose2d(Constants.DynamicPassing.getTarget(getMt2Pose2d().getTranslation()), new Rotation2d()));
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = DriveState.DEFAULT;
    }
    switch (systemState) {
      case DEFAULT:
        // if (OI.getPOVDown()) {
        // snakeDrive();
        // } else {
        teleopDrive();
        // }
        break;
      case DEFAULT_SLOW:
        // if (OI.getPOVDown()) {
        // snakeDrive();
        // } else {
        teleopDrive();
        // }
        break;
      case DEFAULT_SLOWISH:
        // if (OI.getPOVDown()) {
        // snakeDrive();
        // } else {
        teleopDrive();
        // }
        break;
      case IDLE:

        break;
      case IDLE_SLOW:
        break;
      case SNAKE:
        // if (Math.sqrt(Math.pow(OI.getDriverLeftX(), 2) +
        // Math.pow(OI.getDriverLeftY(), 2)) < 0.1) {
        // snakeDrive();
        teleopDrive();
        // } else {
        // snakeDrive();
        // }
        break;
      case STOP:
        stop();
        break;
      case DRIVE_TO_PRE_CLIMB:
        Pose2d climbPrepSetpoint = getClimbPrepSetpoint();
        Logger.recordOutput("climb prep", climbPrepSetpoint);
        driveToPoint(climbPrepSetpoint);
        break;
      case DRIVE_TO_ALIGN_CLIMB:
        Pose2d climbAlignSetpoint = getClimbAlignSetpoint();
        Logger.recordOutput("climb align", climbAlignSetpoint);
        driveToPoint(climbAlignSetpoint);
        break;
      case DRIVE_TO_ALIGN_CLIMB_FINISH:
        autoRobotCentricDrive(new Vector(0, 0.67), 0.0);
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
