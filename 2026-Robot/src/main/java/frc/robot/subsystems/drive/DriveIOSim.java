package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.tools.math.Vector;

public class DriveIOSim extends DriveIO {
    private Vector velocityVector = new Vector(0, 0);
    private Vector positionVector = new Vector(0, 0);
    private Vector wantedVelocityVector = new Vector(0, 0);
    private double angle = 0; // radians
    private double angularVelocity = 0; // radians per second
    private double wantedAngularVelocity = 0; // radians per second squared

    @Override
    void zeroIMU() {
        angle = 0;
    }

    @Override
    void setYaw(double degrees) {
        angle = Math.toRadians(degrees);
    }

    @Override
    Rotation2d getYaw() {
        return new Rotation2d(angle);
    }

    @Override
    void setWheelsStraight() {
    }

    @Override
    protected void setCurrentLimits(int supply, int stator) {
    }

    @Override
    protected void setPosition(Pose2d pose) {
        positionVector = new Vector(pose.getX(), pose.getY());
        angle = pose.getRotation().getRadians();
    }

    @Override
    protected Pose2d getPosition() {
        return new Pose2d(positionVector.getI(), positionVector.getJ(), new Rotation2d(angle));
    }

    @Override
    protected void drive(Vector velocityVector, double turnVelocity) {
        wantedVelocityVector = velocityVector.flipY();
        wantedAngularVelocity = -turnVelocity;
    }

    @Override
    protected void driveRobotCentric(Vector velocityVector, double turnRadiansPerSec) {
        wantedVelocityVector = velocityVector.rotate(angle).flipY();
        wantedAngularVelocity = -turnRadiansPerSec;
    }

    @Override
    protected void driveCamCentric(Vector velocityVector, double turnRadiansPerSec, double camAngle) {
        wantedVelocityVector = velocityVector.rotate(camAngle).flipY();
        wantedAngularVelocity = -turnRadiansPerSec;
    }

    @Override
    protected Vector getVelocityVector() {
        return velocityVector;
    }

    @Override
    void update(DriveState currentState) {
        int numSteps = (int) Math.floor(Globals.loopPeriodSecs / Constants.closedLoopSimResolution);
        double dt = Globals.loopPeriodSecs / numSteps;
        for (int i = 0; i < numSteps; i++) {
            Vector acceleration = wantedVelocityVector.subtract(velocityVector).unit()
                    .scaled(Constants.Physical.SIM_MAX_ACCELERATION * 10);
            velocityVector = velocityVector.add(acceleration.scaled(dt));
            if (velocityVector.magnitude() > Constants.Physical.SIM_TOP_SPEED) {
                velocityVector = velocityVector.scaled(Constants.Physical.SIM_TOP_SPEED / velocityVector.magnitude());
            }
            positionVector = positionVector.add(velocityVector.scaled(dt));
            double angularAcceleration = Math.signum(wantedAngularVelocity - angularVelocity)
                    * Constants.Physical.SIM_MAX_ANGULAR_ACCELERATION;
            angularVelocity += angularAcceleration * dt;
            angle += angularVelocity * dt;
        }
    }

    @Override
    protected double getAngularVelocity() {
        return angularVelocity;
    }

    @Override
    protected Vector getAccelerationVector() {
        return new Vector(0.0, 0.0);
    }
}
