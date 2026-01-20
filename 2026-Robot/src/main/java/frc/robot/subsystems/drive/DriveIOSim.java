package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    protected ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
                velocityVector.getI(),
                velocityVector.getJ(),
                angularVelocity);
    }

    @Override
    void update(DriveState currentState) {
        ChassisSpeeds expectedSpeeds = Constants.Physical.getExpectedDriveSpeeds(Globals.loopPeriodSecs,
                getChassisSpeeds(),
                new ChassisSpeeds(wantedVelocityVector.getI(), wantedVelocityVector.getJ(), wantedAngularVelocity));
        velocityVector = new Vector(expectedSpeeds.vxMetersPerSecond, expectedSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Robot Actual Simmed Velocity", velocityVector.magnitude());
        angularVelocity = expectedSpeeds.omegaRadiansPerSecond;
        positionVector = positionVector.add(velocityVector.scaled(Globals.loopPeriodSecs));
        angle += angularVelocity * Globals.loopPeriodSecs;
    }

    @Override
    protected Vector getAccelerationVector() {
        return new Vector(0.0, 0.0);
    }
}
