package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

interface ShooterIO {
    void updateInputs(ShooterState currentState);

    Rotation2d getHoodAngle();

    Rotation2d getTurretAngle();

    double getFlywheelRPM();

    void moveHoodToAngle(Rotation2d angle);

    void setTurretAngle(double angle);

    void setFlywheelRPM(double rpm);

    void setFlywheelPercent(double percent);

    double getRelativeTurretAngleRadians();

    void setHoodAngle(Rotation2d angle);

    void zeroTurretToEncoder();

    double getFlywheelCurrent();

    double getFlywheelAcceleration();

    double getHoodCurrent();

    double getTurretCurrent();
}
