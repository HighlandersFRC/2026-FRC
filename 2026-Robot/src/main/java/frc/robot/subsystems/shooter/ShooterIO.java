package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

interface ShooterIO {

    void init();

    void updateInputs();

    Rotation2d getHoodAngle();

    Rotation2d getTurretAngle();

    double getFlywheelRPM();

    void setHoodAngle(Rotation2d angle);

    void setTurretAngle(Rotation2d angle);

    void setFlywheelRPM(double rpm);

    double getRelativeTurretAngleRadians();

    double getShooterStatorCurrent();

    void setShooterPercent(double percent);

    void setHoodPercent(double percent);

    double getHoodVelocity();

    double getHoodCurrent();

}
