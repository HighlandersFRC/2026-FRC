package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.tools.math.Vector;

abstract class DriveIO {
    abstract void zeroIMU();

    abstract void setYaw(double degrees);

    abstract Rotation2d getYaw();

    abstract void setWheelsStraight();

    protected abstract void setCurrentLimits(int supply, int stator);

    protected abstract void setPosition(Pose2d pose);

    protected abstract Pose2d getPosition();

    protected abstract void drive(Vector velocityVector, double turnVelocity);

    protected abstract void driveRobotCentric(Vector velocityVector, double turnRadiansPerSec);

    protected abstract void driveCamCentric(Vector velocityVector, double turnRadiansPerSec, double camAngle);

    protected abstract ChassisSpeeds getChassisSpeeds();

    abstract void update(DriveState currentState);
}