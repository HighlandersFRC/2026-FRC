package frc.robot.commands;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;

public class VariableSpeedFollower extends AutoFollower {
  private Drive drive;

  private JSONArray path;

  private double odometryFusedX = 0;
  private double odometryFusedY = 0;
  private double odometryFusedTheta = 0;

  private Number[] desiredVelocityArray = new Number[4];
  private double desiredThetaChange = 0;

  public double pathStartTime;

  private int currentPathPointIndex = 0;
  private int returnPathPointIndex = 0;
  private int timesStagnated = 0;
  private final int STAGNATE_THRESHOLD = Constants.Autonomous.STAGNATE_THRESHOLD;
  private boolean reset = true;
  private int endIndex = 0;

  public int getPathPointIndex() {
    return currentPathPointIndex;
  }

  public VariableSpeedFollower(Drive drive, JSONArray pathPoints,
      boolean record) {
    this.drive = drive;
    if (pathPoints != null) {
      this.path = pathPoints;
      pathStartTime = pathPoints.getJSONObject(0).getDouble("time");
    }
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pathStartTime = path.getJSONObject(0).getDouble("time");
    if (reset) {
      this.endIndex = path.length() - 1;
      currentPathPointIndex = 0;
    } else {
      reset = true;
    }
    if (endIndex > path.length() - 1) {
      endIndex = path.length() - 1;
    }
    returnPathPointIndex = currentPathPointIndex;
    timesStagnated = 0;
  }

  @Override
  public void execute() {
    // System.out.println("Variable Speed");
    odometryFusedX = drive.getMt2Pose2dX();
    odometryFusedY = drive.getMt2Pose2dY();
    odometryFusedTheta = drive.getMt2Pose2dAngle();
    // call PIDController function
    currentPathPointIndex = returnPathPointIndex;
    desiredVelocityArray = drive.purePursuitController(odometryFusedX, odometryFusedY, odometryFusedTheta,
        currentPathPointIndex, path, false, false);

    returnPathPointIndex = desiredVelocityArray[3].intValue();
    if (returnPathPointIndex == currentPathPointIndex && returnPathPointIndex != path.length() - 1) {
      timesStagnated++;
      if (timesStagnated > STAGNATE_THRESHOLD) {
        returnPathPointIndex += Constants.Autonomous.STAGNATE_BOOST;
        if (returnPathPointIndex > endIndex) {
          returnPathPointIndex = endIndex;
        }
        timesStagnated = 0;
      }
    } else {
      timesStagnated = 0;
    }

    Vector velocityVector = new Vector();

    if (currentPathPointIndex == path.length() - 1) {
      velocityVector.setI(desiredVelocityArray[0].doubleValue() * 2);
      velocityVector.setJ(desiredVelocityArray[1].doubleValue() * 2);
      desiredThetaChange = desiredVelocityArray[2].doubleValue() * 2;
    } else if (currentPathPointIndex < 100) {
      velocityVector.setI(desiredVelocityArray[0].doubleValue() * 2);
      velocityVector.setJ(desiredVelocityArray[1].doubleValue() * 2);
      desiredThetaChange = desiredVelocityArray[2].doubleValue() * 2;
    } else {
      velocityVector.setI(desiredVelocityArray[0].doubleValue());
      velocityVector.setJ(desiredVelocityArray[1].doubleValue());
      desiredThetaChange = desiredVelocityArray[2].doubleValue();
    }

    // create velocity vector and set desired theta change

    drive.autoDrive(velocityVector, desiredThetaChange);
    Logger.recordOutput("Auto/pursuing?", true);
    // Logger.recordOutput("Auto/Path Time", path
    // .getJSONObject(getPathPointIndex()).getDouble("time"));
  }

  @Override
  public void end(boolean interrupted) {
    // if (!interrupted) {
    drive.stop();
    // }
    Logger.recordOutput("Auto/pursuing?", false);
  }

  public void from(int pointIndex, JSONObject pathJSON, int to) {
    this.currentPathPointIndex = pointIndex;
    path = pathJSON.getJSONArray("sampled_points");
    endIndex = to;
    reset = false;
  }

  @Override
  public boolean isFinished() {
    boolean readyToEnd = readyToEnd(path.getJSONObject(returnPathPointIndex));
    Logger.recordOutput("Auto/readyToEnd", readyToEnd);
    if (returnPathPointIndex >= path.length() - 1 && readyToEnd) {
      return true;
    } else {
      return false;
    }
  }

  private boolean readyToEnd(JSONObject point) {
    double odometryFusedX = drive.getMt2Pose2dX();
    double odometryFusedY = drive.getMt2Pose2dY();
    double odometryFusedTheta = drive.getMt2Pose2dAngle();
    if (drive.getFieldSide() == "blue") {
      odometryFusedX = Constants.Physical.FIELD_LENGTH - odometryFusedX;
      odometryFusedY = Constants.Physical.FIELD_WIDTH - odometryFusedY;
      odometryFusedTheta = Math.PI + odometryFusedTheta;
    }

    if (OI.isLeftSide()) {
      odometryFusedY = Constants.Physical.FIELD_WIDTH - odometryFusedY;
      odometryFusedTheta = -odometryFusedTheta;
    }
    return drive.insideRadius(
        (point.getDouble("x") - odometryFusedX) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        (point.getDouble("y") - odometryFusedY) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
        (new Rotation2d(point.getDouble("angle")).minus(new Rotation2d(odometryFusedTheta)).getRadians())
            / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
        Constants.Autonomous.AUTONOMOUS_END_ACCURACY) && drive.isFlat();
  }
}