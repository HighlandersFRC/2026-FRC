package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.geometry.Rotation2d;

abstract class ShooterIO {
    protected abstract void shoot(Vector3D initialVelocity);

    protected abstract void updateInputs();

    protected abstract boolean readyToShoot();

    protected abstract Rotation2d getHoodAngle();

    protected abstract Rotation2d getTurretAngle();

    protected abstract double getFlywheelRPM();

    protected abstract void setHoodAngle(Rotation2d angle);

    protected abstract void setTurretAngle(Rotation2d angle);

    protected abstract void setFlywheelRPM(double rpm);

    protected abstract double getRelativeTurretAngleRadians();
}
