// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public enum ShooterState {
    DEFAULT,
    IDLE,
    SHOOT,
    MANUAL_SHOOT,
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;
  private Translation3d _trajectorySetpoint = new Translation3d(0, 0, 0);
  private Rotation2d manualTurretAngle = new Rotation2d(0),
      manualHoodAngle = Rotation2d.fromRadians(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS);
  private double manualFlywheelRPM = 1000.0;

  public Shooter() {
    if (RobotBase.isReal()) {
      this.io = new ShooterIOComp();
    } else {
      this.io = new ShooterIOSim();
    }
  }

  public void setWantedState(ShooterState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(ShooterState wantedState, Translation3d trajectorySetpoint) {
    this.wantedState = wantedState;
    this._trajectorySetpoint = trajectorySetpoint;
  }

  public void setWantedState(ShooterState wantedState, Rotation2d turretAngle, Rotation2d hoodAngle,
      double flywheelRPM) {
    // System.out.println("Setting Manual Shooter State: Hood Angle: " +
    // hoodAngle.getDegrees() + " Turret Angle: "
    // + turretAngle.getDegrees() + " Flywheel RPM: " + flywheelRPM);
    this.wantedState = wantedState;
    this.manualTurretAngle = turretAngle;
    this.manualHoodAngle = hoodAngle;
    this.manualFlywheelRPM = flywheelRPM;
  }

  private ShooterState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ShooterState.DEFAULT;
      case IDLE:
        return ShooterState.IDLE;
      case SHOOT:
        return ShooterState.SHOOT;
      case MANUAL_SHOOT:
        return ShooterState.MANUAL_SHOOT;
      default:
        return ShooterState.IDLE;
    }
  }

  private void shoot() {
    // moveHoodToAngle(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint));
    // setTurretAngle(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));
    setFlywheelRPM(Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
  }

  private void manualShoot() {
    // System.out.println("Manual Shooting: Hood Angle: " +
    // manualHoodAngle.getDegrees() + " Turret Angle: "
    // + manualTurretAngle.getDegrees() + " Flywheel RPM: " + manualFlywheelRPM);
    // moveHoodToAngle(manualHoodAngle);
    // setTurretAngle(manualTurretAngle);
    setFlywheelRPM(manualFlywheelRPM);
  }

  public boolean readyToShoot() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint))
            .getRadians());
    double turretAngleError = Math.abs(
        getRobotRelativeTurretAngle()
            .minus(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint))
            .getRadians());
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
    return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
        && turretAngleError < Constants.SetPoints.Turret.TURRET_PRECISION
        && flywheelRPMError < Constants.SetPoints.Flywheel.FLYWHEEL_RPM_PRECISION;
  }

  public Rotation2d getHoodAngle() {
    return io.getHoodAngle();
  }

  public Rotation2d getRobotRelativeTurretAngle() {
    return io.getTurretAngle();
  }

  public double getRelativeRobotRelativeTurretAngle() {
    return io.getRelativeTurretAngleRadians();
  }

  public double getFlywheelRPM() {
    return io.getFlywheelRPM();
  }

  public void moveHoodToAngle(Rotation2d angle) {
    io.moveHoodToAngle(angle);
  }

  public void setHoodAngle(Rotation2d angle) {
    io.setHoodAngle(angle);
  }

  public void setTurretAngle(Rotation2d angle) {
    io.setTurretAngle(getRelativeAngleFromRotation2d(angle));
  }

  public void setFlywheelRPM(double rpm) {
    io.setFlywheelRPM(rpm);
  }

  public Translation3d getCurrentShooterTrajectory() {
    double mag = io.getFlywheelRPM() / Constants.Physical.Shooter.SHOOTER_WHEEL_RADIUS;
    Rotation2d hoodAngle = io.getHoodAngle();
    double vz = mag * hoodAngle.getSin();
    double vr = mag * hoodAngle.getCos();
    Rotation2d turretAngle = io.getTurretAngle();
    double vx = vr * turretAngle.getCos();
    double vy = vr * turretAngle.getSin();
    return new Translation3d(vx, vy, vz);
  }

  protected double getRelativeAngleFromRotation2d(Rotation2d angle) {
    double relativeAngle = angle.getRadians();
    double currentTurretAngle = io.getRelativeTurretAngleRadians();
    double delta = relativeAngle - currentTurretAngle;
    delta %= 2 * Math.PI;
    if (delta > Math.PI) {
      delta -= 2 * Math.PI;
    } else if (delta < -Math.PI) {
      delta += 2 * Math.PI;
    }
    double wanted = currentTurretAngle + delta;
    return clampAngleToTurretRange(wanted);
  }

  protected double clampAngleToTurretRange(double radians) {
    double clampedAngle = radians;
    while (clampedAngle < -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
      clampedAngle += 2 * Math.PI;
    }
    while (clampedAngle > Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
      clampedAngle -= 2 * Math.PI;
    }
    if (clampedAngle < -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
      clampedAngle = -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
    }
    if (clampedAngle > Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
      clampedAngle = Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
    }
    return clampedAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    Logger.recordOutput("Hood SP",
        Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint).getDegrees());
    Logger.recordOutput("Turret SP",
        Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint).getDegrees());
    Logger.recordOutput("Flywheel RPM SP", Constants.SetPoints.Flywheel
        .getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
    Logger.recordOutput("Hood Angle", getHoodAngle().getDegrees());
    Logger.recordOutput("Turret Angle", getRobotRelativeTurretAngle().getDegrees());
    Logger.recordOutput("Flywheel RPM", getFlywheelRPM());
    Logger.recordOutput("Ready to Shoot", readyToShoot());

    Logger.recordOutput("Shooter State", systemState);
    ShooterState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    switch (systemState) {
      case DEFAULT:
        break;
      case IDLE:
        break;
      case SHOOT:
        shoot();
        break;
      case MANUAL_SHOOT:
        manualShoot();
        break;
      default:
        break;
    }
  }
}
