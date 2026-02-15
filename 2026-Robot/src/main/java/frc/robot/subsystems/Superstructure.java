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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightsState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.tools.logging.TunableNumber;
import frc.robot.tools.math.PhysicsModel;
import frc.robot.tools.math.ShotCalculator;
import frc.robot.tools.math.ShotCalculator.ShotSolution;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Lights lights;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;
  private final Climber climber;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private ArrayList<Translation3d> trajectoryPoint = new ArrayList<Translation3d>();
  private ArrayList<Translation3d> trajectoryVelocity = new ArrayList<Translation3d>();
  private TunableNumber manualShootRPM = new TunableNumber("Manual Shoot RPM", 2000);
  private TunableNumber manualShootHoodAngle = new TunableNumber("Manual Shoot Hood Angle", 60.0);
  private TunableNumber manualShootTurretAngle = new TunableNumber("Manual Shoot Turret Angle", 0.0);

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
    INTAKING,
    SHOOTING,
    SHOOTING_NO_FEED,
    PASS,
    PASSING,
    ZERO,
    MANUAL_SHOOT,
    MANUAL_SHOOTING,
    MANUAL_PASS, // TODO: implement ts and passing
    MANUAL_PASSING,
    MANUAL_CLIMBING,
    MANUAL_EXTEND_CLIMBER,
    AUTON_PREP_SHOT,
    AUTON_SHOOT,
    AUTO_PREP_CLIMB,
    AUTO_ALIGN_CLIMB,
    AUTO_CLIMB,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive,
      Lights lights, Shooter shooter, Intake intake, Feeder feeder, Climber climber) {
    this.drive = drive;
    this.lights = lights;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
    this.climber = climber;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
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
      case INTAKING:
        handleIntakeingState();
        break;
      case ZERO:
        handleZeroState();
        break;
      case MANUAL_CLIMBING:
        handleClimbingState();
        break;
      case MANUAL_EXTEND_CLIMBER:
        handleExtendClimberState();
        break;
      case AUTON_PREP_SHOT:
        handleAutonPrepShot();
        break;
      case AUTON_SHOOT:
        handleAutonShot();
        break;
      case AUTO_PREP_CLIMB:
        handleAutoPrepClimb();
        break;
      case AUTO_ALIGN_CLIMB:
        handleAutoAlignClimb();
        break;
      case AUTO_CLIMB:
        handleAutoClimb();
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
        if (Constants.Field.isOnBump(drive.getMt2Pose2d().getTranslation())) {
          if (DriverStation.isAutonomous()) {
            currentSuperState = SuperState.IDLE;
          } else {
            currentSuperState = SuperState.DEFAULT;
          }
          break;
        }
        if (Constants.Field.isInAllianceZone(drive.getMt2Pose2d().getTranslation())) {
          if (shooter.readyToShoot()) {
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
      case PASS:
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case MANUAL_SHOOT:
        if (OI.driverA.getAsBoolean()) {
          currentSuperState = SuperState.MANUAL_SHOOTING;
        } else {
          currentSuperState = SuperState.MANUAL_SHOOT;
        }
        break;
      case MANUAL_SHOOTING:
        currentSuperState = SuperState.MANUAL_SHOOTING;
        break;
      case INTAKING:
        currentSuperState = SuperState.INTAKING;
        break;
      case SHOOTING:
        currentSuperState = SuperState.SHOOTING;
        break;
      case SHOOTING_NO_FEED:
        currentSuperState = SuperState.SHOOTING_NO_FEED;
        break;
      case PASSING:
        currentSuperState = SuperState.PASSING;
        break;
      case ZERO:
        currentSuperState = SuperState.ZERO;
        break;
      case MANUAL_CLIMBING:
        currentSuperState = SuperState.MANUAL_CLIMBING;
        break;
      case MANUAL_EXTEND_CLIMBER:
        currentSuperState = SuperState.MANUAL_EXTEND_CLIMBER;
        break;
      case AUTON_PREP_SHOT:
        currentSuperState = SuperState.AUTON_PREP_SHOT;
        break;
      case AUTO_PREP_CLIMB:
        if (drive.hitSetPoint(drive.getClimbPrepSetpoint())
            && climber.getClimberPosition() > Constants.Ratios.Climber.CLIMBER_MAX_ROTATIONS - 5.0) {
          wantedSuperState = SuperState.AUTO_ALIGN_CLIMB;
          currentSuperState = SuperState.AUTO_ALIGN_CLIMB;
        } else {
          currentSuperState = SuperState.AUTO_PREP_CLIMB;
        }
        break;
      case AUTO_ALIGN_CLIMB:
        if (drive.hitSetPoint(drive.getClimbAlignSetpoint())) {
          wantedSuperState = SuperState.AUTO_CLIMB;
          currentSuperState = SuperState.AUTO_CLIMB;
        } else {
          currentSuperState = SuperState.AUTO_ALIGN_CLIMB;
        }
        break;
      case AUTO_CLIMB:
        currentSuperState = SuperState.AUTO_CLIMB;
        break;
      case AUTON_SHOOT:
        currentSuperState = SuperState.AUTON_SHOOT;
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
        drive.getChassisSpeeds());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);

    // Feeder
    feeder.setWantedState(FeederState.FEED);
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
        drive.getChassisSpeeds());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter

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
    intake.setWantedState(IntakeState.INTAKING);
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
        drive.getChassisSpeeds());
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
    intake.setWantedState(IntakeState.UP);
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
        Constants.Field.getFeedTarget(turret.toTranslation2d()),
        drive.getChassisSpeeds());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.FEED);
    intake.setWantedState(IntakeState.INTAKING);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  private void handlePassingState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution = ShotCalculator.calculateFeedShot(
        new Pose2d(turret.toTranslation2d(), drive.getMt2Pose2d().getRotation()),
        Constants.Field.getFeedTarget(turret.toTranslation2d()),
        drive.getChassisSpeeds());
    ShotSolution rotatedShotSolution = shotSolution.rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        rotatedShotSolution);
    // Feeder
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter

    intake.setWantedState(IntakeState.INTAKING);
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE_SLOW);
    } else {
      drive.setWantedState(DriveState.DEFAULT_SLOW);
    }
  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT); // Run hopper and linearizer
    intake.setWantedState(IntakeState.UP);
    shooter.setWantedState(ShooterState.DEFAULT);
    climber.setWantedState(ClimberState.IDLE);
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
    Translation2d turret2d = initial.toTranslation2d();
    Translation2d target = Constants.Field.getFeedTarget(turret2d);
    Logger.recordOutput("Shooter/feed target", target);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        new Rotation2d(Math.toRadians(manualShootTurretAngle.get())), distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.HOP); // Run Hopper Only
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
  }

  public void handleManualShootingState() { // TODO: not actual manual shooting
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation2d turret2d = initial.toTranslation2d();
    Translation2d target = Constants.Field.getFeedTarget(turret2d);
    Translation2d hub = target;
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    ShotSolution shotSolution = new ShotSolution(new Rotation2d(Math.toRadians(manualShootHoodAngle.get())),
        manualShootRPM.get(),
        new Rotation2d(Math.toRadians(manualShootTurretAngle.get())), distance2D, 2.0);
    shooter.setWantedState(ShooterState.NORMAL_SHOOT,
        shotSolution);
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
  }

  public void handleIntakeingState() {
    intake.setWantedState(IntakeState.INTAKING);
    feeder.setWantedState(FeederState.HOP); // Run hopper and linearizer
    if (DriverStation.isAutonomous()) {
      drive.setWantedState(DriveState.IDLE);
    } else {
      drive.setWantedState(DriveState.DEFAULT);
    }
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
    lights.setWantedState(LightsState.DEFAULT);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE); // Run hopper and linearizer
    shooter.setWantedState(ShooterState.DEFAULT);
  }

  public void handleZeroState() {
    drive.setWantedState(DriveState.IDLE);
    lights.setWantedState(LightsState.DEFAULT);
    intake.setWantedState(IntakeState.IDLE);
    feeder.setWantedState(FeederState.IDLE); // Run hopper and linearizer
    // shooter.setWantedState(ShooterState.ZERO); TODO: Implement zeroing
  }

  private void handleClimbingState() {
    climber.setWantedState(ClimberState.CLIMBING);
  }

  private void handleExtendClimberState() {
    climber.setWantedState(ClimberState.EXTEND);
  }

  private void handleAutonPrepShot() {
    drive.setWantedState(DriveState.IDLE);
    shooter.setWantedState(ShooterState.PHYSICS_SHOOT);
    feeder.setWantedState(FeederState.FEED);
  }

  private void handleAutonShot() {
    drive.setWantedState(DriveState.IDLE);
    shooter.setWantedState(ShooterState.PHYSICS_SHOOT);
    feeder.setWantedState(FeederState.SHOOT);
  }

  private void handleAutoPrepClimb() {
    drive.setWantedState(DriveState.DRIVE_TO_PRE_CLIMB);
    climber.setWantedState(ClimberState.EXTEND);
    if (DriverStation.isAutonomous()) {
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(getTurretFieldPosition().toTranslation2d(),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getChassisSpeeds());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());

      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.SHOOT);
      } else {
        feeder.setWantedState(FeederState.FEED);
      }
    }
  }

  private void handleAutoAlignClimb() {
    drive.setWantedState(DriveState.DRIVE_TO_ALIGN_CLIMB);
    climber.setWantedState(ClimberState.EXTEND);
    if (DriverStation.isAutonomous()) {
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(getTurretFieldPosition().toTranslation2d(),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getChassisSpeeds());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus());
      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.SHOOT);
      } else {
        feeder.setWantedState(FeederState.FEED);
      }
    }
  }

  private void handleAutoClimb() {
    drive.setWantedState(DriveState.STOP);
    climber.setWantedState(ClimberState.CLIMBING);
    if (DriverStation.isAutonomous()) {
      ShotSolution shotSolution = ShotCalculator.calculateHubShot(getTurretFieldPosition().toTranslation2d(),
          Constants.Field.getHubPose().toTranslation2d(),
          drive.getChassisSpeeds());
      ShotSolution rotatedShotSolution = shotSolution
          .rotateTurretAngle(drive.getMt2Pose2d().getRotation().unaryMinus()).addRPM(-80.0);
      shooter.setWantedState(ShooterState.NORMAL_SHOOT,
          rotatedShotSolution);
      if (shooter.readyToShoot()) {
        feeder.setWantedState(FeederState.SHOOT);
      } else {
        feeder.setWantedState(FeederState.FEED);
      }
    }
  }

  public void PARTY() {
    lights.PARTY();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/turret field pose", new Pose3d(getTurretFieldPosition(),
        new Rotation3d(drive.getMt2Pose2d().getRotation().plus(shooter.getRobotRelativeTurretAngle()))));
    PARTY();
    Rotation2d turret = Constants.Field.getHubPose().toTranslation2d().minus(drive.getMt2Pose2d().getTranslation())
        .getAngle();
    turret = turret.minus(drive.getMt2Pose2d().getRotation());
    shooter.passIdleTurretAngleToIdle(turret);

    currentSuperState = handleStateTransitions();
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
          Logger.recordOutput("Fuel/" + i, trajectoryPoint.get(i));
        }
      }
    }
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    Logger.recordOutput("States/Super State", currentSuperState);
    Logger.recordOutput("Shooter/Manual Shoot RPM", manualShootRPM.get());
    Logger.recordOutput("Shooter/Manual Shoot Hood Angle", manualShootHoodAngle.get());
    Logger.recordOutput("Shooter/Manual Shoot Turret Angle", manualShootTurretAngle.get());
    Logger.recordOutput("Shooter/Ready to Shoot", shooter.readyToShoot());
    applyStates();

  }
}