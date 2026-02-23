package frc.robot.commands;

import org.json.JSONArray;
import org.json.JSONObject;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.AutoFollower;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AccurateFollower extends AutoFollower {
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
    private final int STAGNATE_THRESHOLD = 3;
    private boolean reset = true;
    private int endIndex = 0;

    public int getPathPointIndex() {
        return currentPathPointIndex;
    }

    public AccurateFollower(Drive drive) {
        this.drive = drive;
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
                currentPathPointIndex, path, false, true);

        returnPathPointIndex = desiredVelocityArray[3].intValue();
        if (returnPathPointIndex == currentPathPointIndex && returnPathPointIndex != path.length() - 1) {
            timesStagnated++;
            if (timesStagnated > STAGNATE_THRESHOLD) {
                returnPathPointIndex++;
                timesStagnated = 0;
            }
        } else {
            timesStagnated = 0;
        }

        Vector velocityVector = new Vector();

        if (currentPathPointIndex == path.length() - 1) {
            velocityVector.setI(desiredVelocityArray[0].doubleValue() * 6);
            velocityVector.setJ(desiredVelocityArray[1].doubleValue() * 6);
            desiredThetaChange = desiredVelocityArray[2].doubleValue() * 6;
        } else {
            velocityVector.setI(desiredVelocityArray[0].doubleValue());
            velocityVector.setJ(desiredVelocityArray[1].doubleValue());
            desiredThetaChange = desiredVelocityArray[2].doubleValue();
        }

        // create velocity vector and set desired theta change

        drive.autoDrive(velocityVector, desiredThetaChange);
    }

    @Override
    public void end(boolean interrupted) {
        Vector velocityVector = new Vector();
        velocityVector.setI(0);
        velocityVector.setJ(0);
        double desiredThetaChange = 0.0;
        drive.autoDrive(velocityVector, desiredThetaChange);
    }

    public void from(int pointIndex, JSONObject pathJSON, int to) {
        this.currentPathPointIndex = pointIndex;
        path = pathJSON.getJSONArray("sampled_points");
        endIndex = to;
        reset = false;
    }

    @Override
    public boolean isFinished() {
        if (returnPathPointIndex >= path.length() - 1 && readyToEnd(path.getJSONObject(returnPathPointIndex))) {
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
            odometryFusedTheta = Math.PI - odometryFusedTheta;
        }
        odometryFusedTheta = Constants.standardizeAngleToOther(odometryFusedTheta, point.getDouble("angle"));
        return drive.insideRadius(
                (point.getDouble("x") - odometryFusedX) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                (point.getDouble("y") - odometryFusedY) / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                (point.getDouble("angle") - odometryFusedTheta)
                        / Constants.Autonomous.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
                Constants.Autonomous.ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY) && drive.isFlat();
    }
}