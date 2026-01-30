package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

interface ShooterIO {
    void updateInputs();

    Rotation2d getHoodAngle();

    Rotation2d getTurretAngle();

    double getFlywheelRPM();

    void moveHoodToAngle(Rotation2d angle);

    void setTurretAngle(double angle);

    void setFlywheelRPM(double rpm);

    void setFlywheelPercent(double percent);

    double getRelativeTurretAngleRadians();

    void setHoodAngle(Rotation2d angle);
}
