// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.math.ShotCalculator.ShotSolution;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public enum ShooterState {
    DEFAULT,
    IDLE,
    PHYSICS_SHOOT,
    NORMAL_SHOOT,
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;
  private Translation3d _trajectorySetpoint = new Translation3d(0, 0, 0);
  private Rotation2d idleTurretAngle = new Rotation2d(0.0);
  private ShotSolution wantedShotSolution = new ShotSolution(idleTurretAngle, 0.0, idleTurretAngle, 0.0, 0.0);

  Translation3d target = Constants.Field.getHubPose();
  Pose2d turretPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
  Pose2d robotPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
  Translation3d turretFieldPosition = new Translation3d(0.0, 0.0, 0.0);

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

  public void setWantedState(ShooterState wantedState, ShotSolution wantedShotSolution) {
    this.wantedState = wantedState;
    this.wantedShotSolution = wantedShotSolution;
  }

  private ShooterState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ShooterState.DEFAULT;
      case IDLE:
        return ShooterState.IDLE;
      case PHYSICS_SHOOT:
        return ShooterState.PHYSICS_SHOOT;
      case NORMAL_SHOOT:
        return ShooterState.NORMAL_SHOOT;
      default:
        return ShooterState.IDLE;
    }
  }

  private void physicsShoot() {
    moveHoodToAngle(Constants.SetPoints.Hood.getHoodAngleSetpointForTrajectory(_trajectorySetpoint));
    setTurretAngle(Constants.SetPoints.Turret.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));
    setFlywheelRPM(Constants.SetPoints.Flywheel.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
  }

  private void normalShoot() {
    Logger.recordOutput("Shooter/Wanted Turret Angle", wantedShotSolution.turretAngle.getDegrees());
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = Constants.SetPoints.Turret.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.1);
      Logger.recordOutput("Shooter/Predicted Turret Angle",
          prediction.getDegrees());
      setTurretAngle(prediction);
    } else {
      setTurretAngle(wantedShotSolution.turretAngle);

    }
    moveHoodToAngle(wantedShotSolution.hoodAngle);
    setFlywheelRPM(wantedShotSolution.flywheelRPM);
  }

  public boolean readyToShoot() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(wantedShotSolution.hoodAngle)
            .getRadians());
    double turretAngleError;
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = Constants.SetPoints.Turret.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.1);
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(prediction)
              .getRadians());
    } else {
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(wantedShotSolution.turretAngle)
              .getRadians());

    }

    double turretPrecisionRequired = Math
        .atan((Constants.Field.HUB_RADIUS - Constants.Field.BALL_WIDTH) / wantedShotSolution.distanceToTarget);
    Logger.recordOutput("Shooter/Turret Precision Required", Math.toDegrees(turretPrecisionRequired));
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - wantedShotSolution.flywheelRPM);
    return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
        && turretAngleError < turretPrecisionRequired
        && flywheelRPMError < Constants.SetPoints.Flywheel.FLYWHEEL_RPM_PRECISION;
  }

  public boolean readyToPass() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(wantedShotSolution.hoodAngle)
            .getRadians());
    double turretAngleError;
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = Constants.SetPoints.Turret.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.2);
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(prediction)
              .getRadians());
    } else {
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(wantedShotSolution.turretAngle)
              .getRadians());

    }
    double turretPrecisionRequired = Math
        .atan((Constants.Field.FEED_RADIUS - Constants.Field.BALL_WIDTH) / wantedShotSolution.distanceToTarget);
    Logger.recordOutput("Shooter/Turret Precision Required", Math.toDegrees(turretPrecisionRequired));
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - wantedShotSolution.flywheelRPM);
    return hoodAngleError < Constants.SetPoints.Hood.HOOD_PRECISION
        && turretAngleError < turretPrecisionRequired
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

  // public void setHoodAngle(Rotation2d angle) {
  // io.setHoodAngle(angle);
  // }

  public void setTurretAngle(Rotation2d angle) {
    io.setTurretAngle(getRelativeAngleFromRotation2d(angle));
  }

  public void setFlywheelRPM(double rpm) {
    io.setFlywheelRPM(rpm);
  }

  public void zeroTurretToEncoder() {
    io.zeroTurretToEncoder();
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

  public void passIdleTurretAngleToIdle(Rotation2d angle) {
    idleTurretAngle = angle;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    Logger.recordOutput("Shooter/Hood Angle", getHoodAngle().getDegrees());
    Logger.recordOutput("Shooter/Turret Angle", getRobotRelativeTurretAngle().getDegrees());
    Logger.recordOutput("Shooter/Flywheel RPM", getFlywheelRPM());
    Logger.recordOutput("Shooter/Velocity Magnitude",
        Math.sqrt(Math.pow(_trajectorySetpoint.getX(), 2)
            + Math.pow(_trajectorySetpoint.getY(), 2)
            + Math.pow(_trajectorySetpoint.getZ(), 2)));
    Logger.recordOutput("Shooter/Shooter State", systemState);
    Logger.recordOutput("States/Shooter State", systemState);

    ShooterState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    switch (systemState) {
      case DEFAULT:
        trackTarget();
        break;
      case IDLE:
        break;
      case PHYSICS_SHOOT:
        physicsShoot();
        break;
      case NORMAL_SHOOT:
        normalShoot();
        break;
      default:
        break;
    }
  }

  private void trackTarget() {
    io.setTurretAngle(getRelativeAngleFromRotation2d(idleTurretAngle));
    io.setHoodAngle(new Rotation2d(Constants.SetPoints.Hood.HOOD_MAX_ANGLE_RADIANS));
    io.setFlywheelPercent(0.0);
  }
}
