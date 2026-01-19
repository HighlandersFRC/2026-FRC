// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private double zeroTime = Timer.getFPGATimestamp();
  private boolean firstTimeZero = true;
  private Rotation2d shootBasicHoodAngle = new Rotation2d(
      Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS - Math.toRadians(5.0));
  private double shootBasicFlywheelRPM = 900.0;

  public enum ShooterState {
    DEFAULT,
    IDLE,
    SHOOT,
    MANUAL_SHOOT,
    SHOOT_BASIC,
    ZERO,
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;
  private Translation3d _trajectorySetpoint = new Translation3d(0, 0, 0);

  public void init() {
    io.init();
  }

  public Shooter() {
    if (RobotBase.isReal()) {
      this.io = new ShooterIOComp(this);
    } else {
      this.io = new ShooterIOSim(this);
    }
  }

  public void setWantedState(ShooterState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(ShooterState wantedState, Translation3d trajectorySetpoint) {
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
      case SHOOT_BASIC:
        return ShooterState.SHOOT_BASIC;
      case ZERO:
        return ShooterState.ZERO;
      default:
        return ShooterState.IDLE;
    }
  }

  private void shoot() {
    // moveHoodToAngle(Constants.launchAngleToHoodAngle(
    //     Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint), // TODO: uncomment
    //     Constants.shooterMPSToRPM(_trajectorySetpoint.getNorm())));

    // setTurretAngle(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));

    // setFlywheelRPM(
    //     Constants.shooterMPSToRPM(_trajectorySetpoint.getNorm())); // TODO: uncomment
  }

  public double getGoalShootingTheta() {
    return Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint).getDegrees();
  }

  public boolean readyToShoot() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint))
            .getRadians());
    // double turretAngleError = Math.abs(
    // getRobotRelativeTurretAngle()
    // .minus(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint))
    // .getRadians());
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
    return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
        // && turretAngleError < Constants.SetPoints.Turret.TURRET_PRECISION
        && flywheelRPMError < Constants.SetPoints.Flywheel.FLYWHEEL_RPM_PRECISION;
  }

  public boolean readyToShootBasic() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(shootBasicHoodAngle)
            .getRadians());
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - shootBasicFlywheelRPM);
    return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
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
    io.setHoodAngle(angle);
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

  public void setShooterPercent(double percent) {
    io.setShooterPercent(percent);
  }

  public void setHoodPercent(double percent) {
    io.setHoodPercent(percent);
  }

  public boolean isZeroed() {
    if (Math.abs(io.getHoodVelocity()) < 1.0 && Math.abs(io.getHoodCurrent()) > 1.0
        && Timer.getFPGATimestamp() - zeroTime > 1.0) {

      return true;
    } else {
      return false;
    }
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
    // while (clampedAngle <
    // -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
    // clampedAngle += 2 * Math.PI;
    // }
    // while (clampedAngle > Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS)
    // {
    // clampedAngle -= 2 * Math.PI;
    // }
    // if (clampedAngle < -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
    // clampedAngle = -Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
    // }
    // if (clampedAngle > Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS) {
    // clampedAngle = Constants.Physical.Shooter.TURRET_MAX_ROTATION_RADIANS;
    // }
    return clampedAngle;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    // Logger.recordOutput("Hood SP",
    // Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint).getDegrees());
    // Logger.recordOutput("Turret SP",
    // Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint).getDegrees());
    // Logger.recordOutput("Flywheel RPM SP", Constants.SetPoints.Flywheel
    // .getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
    shootBasicHoodAngle = new Rotation2d(
        Math.toRadians(SmartDashboard.getNumber("Angle", shootBasicHoodAngle.getDegrees())));
    shootBasicFlywheelRPM = SmartDashboard.getNumber("RPM", shootBasicFlywheelRPM);
    Logger.recordOutput("Hood Setpoint", shootBasicHoodAngle.getDegrees());
    Logger.recordOutput("Hood Angle", getHoodAngle().getDegrees());
    Logger.recordOutput("Turret Angle", getRobotRelativeTurretAngle().getDegrees());
    Logger.recordOutput("Flywheel RPM", getFlywheelRPM());
    Logger.recordOutput("Ready to Shoot", readyToShoot());
    Logger.recordOutput("Shooter State", systemState);
    ShooterState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    if (systemState != ShooterState.ZERO) {
      firstTimeZero = true;
      zeroTime = Timer.getFPGATimestamp();
    }
    switch (systemState) {
      case DEFAULT:
        setShooterPercent(0.0);
        setHoodPercent(0.0);
        break;
      case IDLE:
        setShooterPercent(0.0);
        setHoodPercent(0.0);
        break;
      case SHOOT:
        shoot();

        // setHoodPercent(0.1);
        break;
      case SHOOT_BASIC:
        setFlywheelRPM(shootBasicFlywheelRPM);
        setHoodAngle(shootBasicHoodAngle);
        // setShooterPercent(0.3);
        // setHoodPercent(-0.1);
        break;
      case ZERO:
        if (firstTimeZero) {
          firstTimeZero = false;
          zeroTime = Timer.getFPGATimestamp();
        }
        setShooterPercent(0.0);
        setHoodPercent(-0.2);
        break;
      default:
        setShooterPercent(0.0);
        setHoodPercent(0.0);
        break;
    }
  }
}
