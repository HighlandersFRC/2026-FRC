package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.tools.logging.TunableNumber;
import frc.robot.tools.math.PhysicsModel;
import frc.robot.tools.math.ShotCalculator;
import frc.robot.tools.math.ShotCalculator.ShotSolution;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  boolean firstTimeAutoClimb = true;
  double alignTime = Timer.getFPGATimestamp();
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private ArrayList<Translation3d> trajectoryPoint = new ArrayList<Translation3d>();
  private ArrayList<Translation3d> trajectoryVelocity = new ArrayList<Translation3d>();
  private TunableNumber manualShootRPM = new TunableNumber("Manual Shoot RPM", 0.0);
  private TunableNumber manualShootHoodAngle = new TunableNumber("Manual Shoot Hood Angle", 0.0);
  // private TunableNumber manualShootTurretAngle = new TunableNumber("Manual
  // Shoot Turret Angle", 0.0);
  private ShotSolution presetShotSolution = new ShotSolution(new Rotation2d(Math.toRadians(60.0)), 2000,
      new Rotation2d(Math.PI),
      0.0, 0.0);
  private boolean inSlowState = false;

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
    SHOOT_NO_JIGGLE,
    INTAKING,
    INTAKING_NO_SLOW,
    SHOOTING,
    SHOOTING_NO_JIGGLE,
    SHOOTING_NO_FEED,
    PASS,
    PASSING,
    ZERO,
    MANUAL_SHOOT,
    MANUAL_SHOOTING,
    PRESET_SHOOT,
    PRESET_SHOOTING,
    MANUAL_PASS,
    MANUAL_PASSING,
    INTAKE_UP,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive, Shooter shooter, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public void setWantedState(SuperState wantedState, ShotSolution shotSolution) {
    this.wantedSuperState = wantedState;
    this.presetShotSolution = shotSolution;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public SuperState getLastSuperState() {
    return lastState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        handleDefaultState();
        break;
      case SHOOT:
        handleShootState();
        break;
      case SHOOTING:
        handleShootingState();
        break;
      case SHOOTING_NO_FEED:
        handleShootingNoFeedState();
        break;
      case SHOOTING_NO_JIGGLE:
        handleShootingNoJiggleState();
        break;
      case SHOOT_NO_JIGGLE:
        handleShootNoJiggleState();
        break;
      case PASS:
        handlePassState();
        break;
      case PASSING:
        handlePassingState();
        break;
      case MANUAL_SHOOT:
        handleManualShootState();
        break;
      case MANUAL_SHOOTING:
        handleManualShootingState();
        break;
      case PRESET_SHOOT:
        handlePresetShootState();
        break;
      case PRESET_SHOOTING:
        handlePresetShootingState();
        break;
      case INTAKING:
        handleIntakeingState();
        break;
      case INTAKING_NO_SLOW:
        handleIntakingNoSlowState();
        break;
      case INTAKE_UP:
        handleIntakeUpState();
        break;
      case ZERO:
        handleZeroState();
        break;
      default:
        handleIdleState();
        break;
    }
  }

  /**
   * This function handles the state transitions of the Superstructure subsystem.
   * It updates the current state based on the wanted state and performs necessary
   * actions.
   *
   * @return SuperState - The current state of the Superstructure subsystem after
   *         handling the state transitions.
   *
   * @param wantedSuperState The desired state of the Superstructure subsystem.
   *
   * @see SuperState
   */
  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case SHOOT:
        if (Constants.Field.isInAllianceZone(drive.getMt2Pose2d().getTranslation())
            || DriverStation.isAutonomousEnabled()) {
          if (shooter.readyToShoot() && !Constants.Field.isOnBump(drive.getMt2Pose2d().getTranslation())) {
            currentSuperState = SuperState.SHOOTING;
          } else {
            currentSuperState = SuperState.SHOOT;
          }
          break;
        }
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case SHOOT_NO_JIGGLE:
        if (Constants.Field.isInAllianceZone(drive.getMt2Pose2d().getTranslation())) {
          if (shooter.readyToShoot() && !Constants.Field.isOnBump(drive.getMt2Pose2d().getTranslation())) {
            currentSuperState = SuperState.SHOOTING_NO_JIGGLE;
          } else {
            currentSuperState = SuperState.SHOOT_NO_JIGGLE;
          }
          break;
        }
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case PASS:
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case MANUAL_SHOOT:
        if (OI.driverY.getAsBoolean()) {
          currentSuperState = SuperState.MANUAL_SHOOTING;
        } else {
          currentSuperState = SuperState.MANUAL_SHOOT;
        }
        break;
      case MANUAL_SHOOTING:
        currentSuperState = SuperState.MANUAL_SHOOTING;
        break;
      case PRESET_SHOOT:
        if (shooter.readyToShoot()) {
          currentSuperState = SuperState.PRESET_SHOOTING;
        } else {
          currentSuperState = SuperState.PRESET_SHOOT;
        }
        break;
      case PRESET_SHOOTING:
        currentSuperState = SuperState.PRESET_SHOOTING;
        break;
      case INTAKING:
        currentSuperState = SuperState.INTAKING;
        break;
      case INTAKING_NO_SLOW:
        currentSuperState = SuperState.INTAKING_NO_SLOW;
        break;
      case INTAKE_UP:
        currentSuperState = SuperState.INTAKE_UP;
        break;
      case SHOOTING:
        currentSuperState = SuperState.SHOOTING;
        break;
      case SHOOTING_NO_FEED:
        currentSuperState = SuperState.SHOOTING_NO_FEED;
        break;
      case SHOOTING_NO_JIGGLE:
        currentSuperState = SuperState.SHOOTING_NO_JIGGLE;
        break;
      case PASSING:
        currentSuperState = SuperState.PASSING;
        break;
      case ZERO:
        if (intake.isZeroed()) {
          intake.setWantedState(IntakeState.DOWN);
          wantedSuperState = SuperState.DEFAULT;
          currentSuperState = SuperState.DEFAULT;
        }
        currentSuperState = SuperState.ZERO;
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  private void handleShootState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);

    // Feeder
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootNoJiggleState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);

    // Feeder
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootingState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d target = Constants.Field.getHubPose();
      Translation3d initial = getTurretFieldPosition();
      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.JIGGLE);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootingNoJiggleState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d target = Constants.Field.getHubPose();
      Translation3d initial = getTurretFieldPosition();
      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.DOWN);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handleShootingNoFeedState() {
    // Shooter
    ShotSolution shotSolution = ShotCalculator.calculateHubShot(
        new Pose2d(getTurretFieldPosition().toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getHubPose().toTranslation2d(),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.DEFAULT); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d target = Constants.Field.getHubPose();
      Translation3d initial = getTurretFieldPosition();
      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.DOWN);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePresetShootState() {
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        presetShotSolution);
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePresetShootingState() {
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        presetShotSolution);
    feeder.setWantedState(FeederState.FEED);
    intake.setWantedState(IntakeState.JIGGLE);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePassState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution = ShotCalculator.calculateFeedShot(
        new Pose2d(turret.toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.DynamicPassing.getTarget(turret
            .toTranslation2d()),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOWISH);
    }
  }

  private void handlePassingState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution = ShotCalculator.calculateFeedShot(
        new Pose2d(turret.toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.DynamicPassing.getTarget(turret.toTranslation2d()),
        drive.getFutureVelocity());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter

    intake.setWantedState(IntakeState.JIGGLE);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOWISH);
    }
  }

  public void handleDefaultState() {
    drive.setWantedState(DriveState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT);
    // intake.setWantedState(IntakeState.DOWN);
    shooter.setWantedState(ShooterState.DEFAULT);
  }

  public Translation3d getTurretFieldPosition() {
    return new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
  }

  public void handleManualShootState() { // TODO: not actually manual shooting
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation2d target = Constants.Field.getHubPose().toTranslation2d();
    // Logger.recordOutput("Shooter/feed target", target);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Rotation2d turret = Constants.Field.getHubPose().toTranslation2d()
        .minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    // Logger.recordOutput("Shooter/Manual Shoot Angle to Hub",
    // turret.getDegrees());
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        turret, distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.DYNAMIC_INTAKING);
  }

  public void handleManualShootingState() { // TODO: not actual manual shooting
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation2d target = Constants.Field.getHubPose().toTranslation2d();
    // Logger.recordOutput("Shooter/feed target", target);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Rotation2d turret = Constants.Field.getHubPose().toTranslation2d()
        .minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    // Logger.recordOutput("Shooter/Manual Shoot Angle to Hub",
    // turret.getDegrees());
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        turret, distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.FEED); // Pass ball into shooter
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.JIGGLE);
  }

  public void handleIntakeingState() {
    shooter.setWantedState(ShooterState.DEFAULT);
    intake.setWantedState(IntakeState.DYNAMIC_INTAKING);
    feeder.setWantedState(FeederState.DEFAULT);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOWISH);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleIntakingNoSlowState() {
    shooter.setWantedState(ShooterState.DEFAULT);
    intake.setWantedState(IntakeState.DYNAMIC_INTAKING);
    feeder.setWantedState(FeederState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleIntakeUpState() {
    intake.setWantedState(IntakeState.UP);
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
    shooter.setWantedState(ShooterState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT);
    intake.setWantedState(IntakeState.DOWN);
  }

  public void handleZeroState() {
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.ZERO);
    feeder.setWantedState(FeederState.DEFAULT);
    shooter.setWantedState(ShooterState.IDLE);
  }

  public void PARTY() {
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Superstructure/turret field pose", new
    // Pose3d(getTurretFieldPosition(),
    // new
    // Rotation3d(drive.getMt2Pose2d().getRotation().plus(shooter.getRobotRelativeTurretAngle()))));
    PARTY();
    Rotation2d turret = Constants.Field.getHubPose().toTranslation2d().minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    shooter.passIdleTurretAngleToIdle(turret, drive.getRobotAngularVelocity());

    currentSuperState = handleStateTransitions();

    if (!DriverStation.isAutonomousEnabled()) {
      if (currentSuperState == SuperState.SHOOT ||
          currentSuperState == SuperState.SHOOT_NO_JIGGLE ||
          currentSuperState == SuperState.SHOOTING ||
          currentSuperState == SuperState.SHOOTING_NO_JIGGLE ||
          currentSuperState == SuperState.SHOOTING_NO_FEED ||
          currentSuperState == SuperState.PASS ||
          currentSuperState == SuperState.PASSING ||
          currentSuperState == SuperState.MANUAL_SHOOT ||
          currentSuperState == SuperState.MANUAL_SHOOTING ||
          currentSuperState == SuperState.PRESET_SHOOT ||
          currentSuperState == SuperState.PRESET_SHOOTING ||
          currentSuperState == SuperState.MANUAL_PASS ||
          currentSuperState == SuperState.MANUAL_PASSING) {
        if (!inSlowState) {
          drive.lowerCurrentLimits();
          inSlowState = true;
        }
      } else {
        if (inSlowState) {
          drive.resetCurrentLimits();
          inSlowState = false;
        }
      }
    }

    if (RobotBase.isSimulation()) {
      for (int i = 0; i < trajectoryVelocity.size(); i++) {
        trajectoryVelocity.set(i, new Translation3d(trajectoryVelocity.get(i).getX(),
            trajectoryVelocity.get(i).getY(),
            trajectoryVelocity.get(i).getZ() - Constants.G * Globals.loopPeriodSecs));
        trajectoryPoint.set(i, trajectoryPoint.get(i).plus(trajectoryVelocity.get(i).times(Globals.loopPeriodSecs)));
        if (trajectoryPoint.get(i).getZ() < 0) {
          trajectoryPoint.remove(i);
          trajectoryVelocity.remove(i);
          i--;
        } else {
          // Logger.recordOutput("Fuel/" + i, trajectoryPoint.get(i));
        }
      }
    }
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    Logger.recordOutput("States/Super State", currentSuperState);
    // Logger.recordOutput("Testing/Manual Shoot RPM", manualShootRPM.get());
    // Logger.recordOutput("Testing/Manual Shoot Hood Angle",
    // manualShootHoodAngle.get());
    // Logger.recordOutput("Testing/Manual Shoot Turret Angle",
    // manualShootTurretAngle.get());
    // Logger.recordOutput("Shooter/Ready to Shoot", shooter.readyToShoot());
    applyStates();

  }
}