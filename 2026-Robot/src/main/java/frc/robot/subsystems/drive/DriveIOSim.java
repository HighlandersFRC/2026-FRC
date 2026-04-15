package frc.robot.subsystems.drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.tools.math.Vector;

public class DriveIOSim extends DriveIO {
    private Vector velocityVector = new Vector(0, 0);
    private Vector positionVector = new Vector(0, 0);
    private Vector wantedVelocityVector = new Vector(0, 0);
    private double angle = 0; // radians
    private double angularVelocity = 0; // radians per second
    private double wantedAngularVelocity = 0; // radians per second squared
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    private final double[] moduleDistances = new double[4];

    @Override
    void zeroIMU() {
        setPosition(new Pose2d(getPosition().getTranslation(), Rotation2d.kZero));
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
    protected void setPosition(Pose2d pose) {
        positionVector = new Vector(pose.getX(), pose.getY());
        angle = pose.getRotation().getRadians();
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(getChassisSpeeds());
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleStates.length];
        for (int i = 0; i < moduleStates.length; i++) {
            modulePositions[i] = new SwerveModulePosition(moduleDistances[i], moduleStates[i].angle);
        }
        RobotState.getInstance().resetPose(pose, modulePositions, Optional.of(new Rotation2d(angle)));
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
    protected void setDriveCurrentLimits(double limit) {
        // Implementation for setting drive current limit in simulation
    }

    @Override
    protected void setAngleCurrentLimits(double limit) {
        // Implementation for setting angle current limit in simulation
    }

    @Override
    void update(DriveState currentState) {
        ChassisSpeeds expectedSpeeds = Constants.Simulation.getExpectedDriveSpeeds(Globals.loopPeriodSecs,
                getChassisSpeeds(),
                new ChassisSpeeds(wantedVelocityVector.getI(), -wantedVelocityVector.getJ(), wantedAngularVelocity));
        velocityVector = new Vector(expectedSpeeds.vxMetersPerSecond, expectedSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Sim/Robot Actual Simmed Velocity", velocityVector.magnitude());
        angularVelocity = expectedSpeeds.omegaRadiansPerSecond;
        positionVector = positionVector.add(velocityVector.scaled(Globals.loopPeriodSecs));
        angle += angularVelocity * Globals.loopPeriodSecs;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(getChassisSpeeds());
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleStates.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleDistances[i] += moduleStates[i].speedMetersPerSecond * Globals.loopPeriodSecs;
            modulePositions[i] = new SwerveModulePosition(moduleDistances[i], moduleStates[i].angle);
        }

        RobotState.getInstance().addOdometryObservation(new RobotState.OdometryObservation(
                Timer.getFPGATimestamp(),
                modulePositions,
                java.util.Optional.empty(),
                java.util.Optional.empty(),
                java.util.Optional.of(new Rotation2d(angle))));
    }

    @Override
    protected ChassisSpeeds getWantedChassisSpeeds() {
        return new ChassisSpeeds(
                wantedVelocityVector.getI(),
                wantedVelocityVector.getJ(),
                wantedAngularVelocity);
    }

    @Override
    protected boolean getFlat() {
        return true;
    }
}
