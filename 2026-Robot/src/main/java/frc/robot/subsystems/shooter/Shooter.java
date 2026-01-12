// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public enum ShooterState {
    DEFAULT,
    IDLE,
    SHOOT,
    MANUAL_SHOOT
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;
  private Vector3D _trajectorySetpoint = new Vector3D(0, 0, 0);

  public Shooter() {
    if (RobotBase.isReal()) {
      this.io = new ShooterIOComp();
    } else {
      this.io = new ShooterIOSim(this);
    }
  }

  public void setWantedState(ShooterState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(ShooterState wantedState, Vector3D trajectorySetpoint) {
    this.wantedState = wantedState;
    this._trajectorySetpoint = trajectorySetpoint;
  }

  private ShooterState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ShooterState.DEFAULT;
      case IDLE:
        return ShooterState.IDLE;
      case SHOOT:
        return ShooterState.SHOOT;
      default:
        return ShooterState.IDLE;
    }
  }

  private void shoot() {
    io.shoot(_trajectorySetpoint);
  }

  public boolean readyToShoot() {
    return io.readyToShoot();
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

  public void setHoodAngle(Rotation2d angle) {
    io.setHoodAngle(angle);
  }

  public void setTurretAngle(Rotation2d angle) {
    io.setTurretAngle(angle);
  }

  public void setFlywheelRPM(double rpm) {
    io.setFlywheelRPM(rpm);
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
      default:
        break;
    }
  }
}
