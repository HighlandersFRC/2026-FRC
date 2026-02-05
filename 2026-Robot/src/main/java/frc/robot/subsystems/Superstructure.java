package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.tools.math.Vector;

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
    ZERO,
    MANUAL_SHOOT,
    MANUAL_SHOOTING,
    MANUAL_CLIMBING,
    MANUAL_EXTEND_CLIMBER,
    AUTON_PREP_SHOT,
    AUTON_SHOOT,
    AUTO_CLIMB
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
    double distance;
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case SHOOT:
        distance = drive.getMt2Pose2d().getTranslation()
            .getDistance(Globals.fieldSide.equals("blue") ? Constants.Field.HUB_POSE_BLUE.toTranslation2d()
                : Constants.Field.HUB_POSE_RED.toTranslation2d());
        if (shooter.readyToShoot(distance)) {
          wantedSuperState = SuperState.SHOOTING;
          currentSuperState = SuperState.SHOOTING;
        } else {
          currentSuperState = SuperState.SHOOT;
        }
        break;
      case MANUAL_SHOOT:
        if (OI.driverA.getAsBoolean()) {
          wantedSuperState = SuperState.MANUAL_SHOOTING;
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
        distance = drive.getMt2Pose2d().getTranslation()
            .getDistance(Globals.fieldSide.equals("blue") ? Constants.Field.HUB_POSE_BLUE.toTranslation2d()
                : Constants.Field.HUB_POSE_RED.toTranslation2d());
        if (shooter.readyToShoot(distance)) {
          wantedSuperState = SuperState.SHOOTING;
          currentSuperState = SuperState.SHOOTING;
        } else {
          currentSuperState = SuperState.SHOOT;
        }
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

  private Translation3d calculateTurretRelativeOnTheMoveTrajectory() {
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    Translation3d target;
    if (Globals.fieldSide.equals("blue")) {
      target = Constants.Field.HUB_POSE_BLUE;
    } else {
      target = Constants.Field.HUB_POSE_RED;
    }
    Rotation2d gyro = drive.getMt2Pose2d().getRotation();
    double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
    double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
    Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
    ChassisSpeeds velocity = drive.getChassisSpeeds();
    Vector robotVelocity = new Vector(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    double angVel = drive.getRobotAngularVelocity();
    double vx = -angVel * (Constants.Physical.Shooter.SHOOTER_POSITION.getY());
    double vy = angVel * (Constants.Physical.Shooter.SHOOTER_POSITION.getX());
    Vector tangentialVelocity = new Vector(vx, vy);
    Vector shooterVelocity = robotVelocity.add(tangentialVelocity);
    Translation3d onTheMove = new Translation3d(initialVelocity.getX() -
        shooterVelocity.getI(),
        initialVelocity.getY() - shooterVelocity.getJ(),
        initialVelocity.getZ());
    Translation3d trajectory = onTheMove;
    Translation3d loggedTrajectory = trajectory.plus(initial);
    Logger.recordOutput("Shooter/Trajectory", loggedTrajectory);
    Logger.recordOutput("Shooter/Turret Position",
        new Pose3d(initial, new Rotation3d(drive.getMt2Pose2d().getRotation())
            .plus(new Rotation3d(shooter.getRobotRelativeTurretAngle()))));
    gyro = gyro.unaryMinus();
    trajectory = new Translation3d( // For Turret, make the0 trajectory robotcentric
        trajectory.getX() * gyro.getCos() - trajectory.getY() * gyro.getSin(),
        trajectory.getX() * gyro.getSin() + trajectory.getY() * gyro.getCos(),
        trajectory.getZ());
    Translation3d realVector = shooter.getCurrentShooterTrajectory();
    gyro = gyro.unaryMinus();
    Translation3d realTrajectory = new Translation3d(
        realVector.getX() * gyro.getCos() - realVector.getY() * gyro.getSin(),
        realVector.getX() * gyro.getSin() + realVector.getY() * gyro.getCos(),
        realVector.getZ());
    Logger.recordOutput("Shooter/Shooter Trajectory",
        realTrajectory);
    return trajectory;
  }

  private void handleShootState() {
    // Shooter
    Translation3d target;
    if (Globals.fieldSide.equals("blue")) {
      target = Constants.Field.HUB_POSE_BLUE;
    } else {
      target = Constants.Field.HUB_POSE_RED;
    }
    ShotSolution solution = ShotCalculator.calculateShot(drive.getMt2Pose2d(), target.toTranslation2d(),
        drive.getChassisSpeeds());
    shooter.setWantedState(ShooterState.MANUAL_SHOOT,
        solution.turretAngleDegrees,
        solution.hoodAngleDegrees,
        solution.flywheelRPM);

    // Feeder
    feeder.setWantedState(FeederState.FEED);
    intake.setWantedState(IntakeState.INTAKING);
    drive.setWantedState(DriveState.SNAKE);
  }

  private void handleShootingState() {
    // Shooter
    Translation3d target;
    if (Globals.fieldSide.equals("blue")) {
      target = Constants.Field.HUB_POSE_BLUE;
    } else {
      target = Constants.Field.HUB_POSE_RED;
    }
    ShotSolution solution = ShotCalculator.calculateShot(drive.getMt2Pose2d(), target.toTranslation2d(),
        drive.getChassisSpeeds());
    shooter.setWantedState(ShooterState.MANUAL_SHOOT,
        solution.turretAngleDegrees,
        solution.hoodAngleDegrees,
        solution.flywheelRPM);
    // Feeder
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter

    // Log Fuel Trajectory
    if (RobotBase.isSimulation()) {
      Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
          .getMt2Pose2dY(), 0.0)
          .plus(
              Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));

      double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
      double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
      Translation3d initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
      trajectoryPoint.add(initial);
      trajectoryVelocity
          .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    }
    intake.setWantedState(IntakeState.INTAKING);
    drive.setWantedState(DriveState.SNAKE);
  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    feeder.setWantedState(FeederState.DEFAULT); // Run hopper and linearizer
    intake.setWantedState(IntakeState.UP);
    shooter.setWantedState(ShooterState.DEFAULT);
    climber.setWantedState(ClimberState.IDLE);
  }

  public void handleManualShootState() {

    Translation3d target;
    if (Globals.fieldSide.equals("blue")) {
      target = Constants.Field.HUB_POSE_BLUE;
    } else {
      target = Constants.Field.HUB_POSE_RED;
    }
    Translation2d hub = target.toTranslation2d();
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    shooter.setWantedState(ShooterState.MANUAL_SHOOT,
        hub.minus(initial.toTranslation2d()).getAngle().minus(drive.getMt2Pose2d().getRotation()),
        Rotation2d.fromDegrees(manualShootHoodAngle.get()),
        manualShootRPM.get());
    feeder.setWantedState(FeederState.HOP); // Run Hopper Only
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
  }

  public void handleManualShootingState() {
    Translation3d target;
    if (Globals.fieldSide.equals("blue")) {
      target = Constants.Field.HUB_POSE_BLUE;
    } else {
      target = Constants.Field.HUB_POSE_RED;
    }
    Translation2d hub = target.toTranslation2d();
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0.0)
        .plus(Constants.Physical.Shooter.SHOOTER_POSITION.rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
    double distance2D = initial.toTranslation2d().getDistance(hub);
    Logger.recordOutput("Shooter/Manual Shoot Distance to Hub", distance2D);
    shooter.setWantedState(ShooterState.MANUAL_SHOOT,
        hub.minus(initial.toTranslation2d()).getAngle().minus(drive.getMt2Pose2d().getRotation()),
        Rotation2d.fromDegrees(manualShootHoodAngle.get()),
        manualShootRPM.get());
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter
    drive.setWantedState(DriveState.DEFAULT);
    intake.setWantedState(IntakeState.INTAKING);
  }

  public void handleIntakeingState() {
    intake.setWantedState(IntakeState.INTAKING);
    feeder.setWantedState(FeederState.HOP); // Run hopper and linearizer
    drive.setWantedState(DriveState.DEFAULT);
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
    shooter.setWantedState(ShooterState.SHOOT);
    feeder.setWantedState(FeederState.FEED);
  }

  private void handleAutonShot() {
    drive.setWantedState(DriveState.IDLE);
    shooter.setWantedState(ShooterState.SHOOT);
    feeder.setWantedState(FeederState.SHOOT);
  }

  private void handleAutoClimb() {
    drive.setWantedState(DriveState.DRIVE_TO_CLIMB);
    climber.setWantedState(ClimberState.EXTEND);
    if (drive.hitSetPoint(Constants.Physical.climbPose)) {
      climber.setWantedState(ClimberState.CLIMBING);
    }
  }

  public void PARTY() {
    lights.PARTY();
  }

  @Override
  public void periodic() {
    PARTY();
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
    double distance = drive.getMt2Pose2d().getTranslation()
        .getDistance(Globals.fieldSide.equals("blue") ? Constants.Field.HUB_POSE_BLUE.toTranslation2d()
            : Constants.Field.HUB_POSE_RED.toTranslation2d());
    Logger.recordOutput("Shooter/Ready to Shoot", shooter.readyToShoot(distance));
    applyStates();

  }
}