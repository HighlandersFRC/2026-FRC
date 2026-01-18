package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
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
import frc.robot.tools.math.PhysicsModel;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Lights lights;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private ArrayList<Translation3d> trajectoryPoint = new ArrayList<Translation3d>();
  private ArrayList<Translation3d> trajectoryVelocity = new ArrayList<Translation3d>();

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
    INTAKING,
    SHOOTING,
    SHOOT_BASIC,
    SHOOTING_BASIC,
    ZERO,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public boolean algaeMode = false;

  public Superstructure(Drive drive,
      Lights lights, Shooter shooter, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.lights = lights;
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
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
      case INTAKING:
        handleIntakeingState();
        break;
      case SHOOT_BASIC:
        handleShootBasicState();
        break;
      case SHOOTING_BASIC:
        handleShootingBasicState();
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
        if (shooter.readyToShoot() &&
            Math.abs(Math.toDegrees(drive.getMt2Pose2dAngle()) - shooter.getGoalShootingTheta()) < Math
                .toDegrees(Constants.SetPoints.Turret.TURRET_PRECISION)) {
          wantedSuperState = SuperState.SHOOTING;
          currentSuperState = SuperState.SHOOTING;
        } else {
          currentSuperState = SuperState.SHOOT;
        }
        break;
      case INTAKING:
        currentSuperState = SuperState.INTAKING;
        break;
      case SHOOTING:
        currentSuperState = SuperState.SHOOTING;
        break;
      case SHOOT_BASIC:
        // if (shooter.readyToShootBasic()) {
        // wantedSuperState = SuperState.SHOOTING_BASIC;
        // currentSuperState = SuperState.SHOOTING_BASIC;
        // } else {
        currentSuperState = SuperState.SHOOT_BASIC;
        // }
        break;
      case SHOOTING_BASIC:
        currentSuperState = SuperState.SHOOTING_BASIC;
        break;
      case ZERO:
        if (shooter.isZeroed()) {
          if (DriverStation.isAutonomousEnabled()) {
            wantedSuperState = SuperState.IDLE;
            currentSuperState = SuperState.IDLE;
          } else {
            wantedSuperState = SuperState.DEFAULT;
            currentSuperState = SuperState.DEFAULT;
          }
        } else {
          currentSuperState = SuperState.ZERO;
        }
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  private void handleShootState() {
    // Shooter
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
    // Vector robotVelocity = drive.getRobotVelocityVector();
    // Translation3d onTheMove = new Translation3d(initialVelocity.getX() -
    // robotVelocity.getI(),
    // initialVelocity.getY() - robotVelocity.getJ(),
    // initialVelocity.getZ());
    Translation3d trajectory = initialVelocity;
    Translation3d loggedTrajectory = trajectory.plus(initial);
    Logger.recordOutput("Trajectory", loggedTrajectory);
    Logger.recordOutput("Turret Position", new Pose3d(initial, new Rotation3d(drive.getMt2Pose2d().getRotation())
        .plus(new Rotation3d(shooter.getRobotRelativeTurretAngle()))));
    gyro = gyro.unaryMinus();
    // trajectory = new Translation3d(
    // trajectory.getX() * gyro.getCos() - trajectory.getY() * gyro.getSin(),
    // trajectory.getX() * gyro.getSin() + trajectory.getY() * gyro.getCos(),
    // trajectory.getZ());
    shooter.setWantedState(ShooterState.SHOOT, trajectory);
    Translation3d realVector = shooter.getCurrentShooterTrajectory();
    gyro = gyro.unaryMinus();
    Translation3d realTrajectory = new Translation3d(
        realVector.getX() * gyro.getCos() - realVector.getY() * gyro.getSin(),
        realVector.getX() * gyro.getSin() + realVector.getY() * gyro.getCos(),
        realVector.getZ());
    Logger.recordOutput("Shooter Trajectory",
        realTrajectory);
    drive.setGoalShootingTheta(shooter.getGoalShootingTheta());
    drive.setWantedState(DriveState.SHOOTING);
    // Logger.recordOutput("Shooter State", "SHOOT");

    // Feeder
    // if (shooter.readyToShoot()) {
    // feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter
    // trajectoryPoint.add(initial);
    // trajectoryVelocity
    // .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(),
    // initialVelocity.getZ()));
    // } else {
    feeder.setWantedState(FeederState.IDLE); // Run hopper and linearizer
    // }
  }

  private void handleShootingState() {
    // Shooter
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
    // Vector robotVelocity = drive.getRobotVelocityVector();
    // Translation3d onTheMove = new Translation3d(initialVelocity.getX() -
    // robotVelocity.getI(),
    // initialVelocity.getY() - robotVelocity.getJ(),
    // initialVelocity.getZ());
    Translation3d trajectory = initialVelocity;
    Translation3d loggedTrajectory = trajectory.plus(initial);
    Logger.recordOutput("Trajectory", loggedTrajectory);
    Logger.recordOutput("Turret Position", new Pose3d(initial, new Rotation3d(drive.getMt2Pose2d().getRotation())
        .plus(new Rotation3d(shooter.getRobotRelativeTurretAngle()))));
    gyro = gyro.unaryMinus();
    // trajectory = new Translation3d(
    // trajectory.getX() * gyro.getCos() - trajectory.getY() * gyro.getSin(),
    // trajectory.getX() * gyro.getSin() + trajectory.getY() * gyro.getCos(),
    // trajectory.getZ());
    shooter.setWantedState(ShooterState.SHOOT, trajectory);
    Translation3d realVector = shooter.getCurrentShooterTrajectory();
    gyro = gyro.unaryMinus();
    Translation3d realTrajectory = new Translation3d(
        realVector.getX() * gyro.getCos() - realVector.getY() * gyro.getSin(),
        realVector.getX() * gyro.getSin() + realVector.getY() * gyro.getCos(),
        realVector.getZ());
    Logger.recordOutput("Shooter Trajectory",
        realTrajectory);
    drive.setGoalShootingTheta(shooter.getGoalShootingTheta());
    drive.setWantedState(DriveState.SHOOTING);
    // Logger.recordOutput("Shooter State", "SHOOT");

    // Feeder
    // if (shooter.readyToShoot()) {
    // feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter
    // trajectoryPoint.add(initial);
    // trajectoryVelocity
    // .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(),
    // initialVelocity.getZ()));
    // } else {
    feeder.setWantedState(FeederState.SHOOT); // Pass ball into shooter
    trajectoryPoint.add(initial);
    trajectoryVelocity
        .add(new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ()));
    // } else {
    // feeder.setWantedState(FeederState.IDLE); // Run hopper and linearizer
    // }
  }

  private void handleShootBasicState() {
    // drive.setGoalShootingTheta(shooter.getGoalShootingTheta());
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOT_BASIC);
    feeder.setWantedState(FeederState.IDLE);
  }

  private void handleShootingBasicState() {
    // drive.setGoalShootingTheta(shooter.getGoalShootingTheta());
    drive.setWantedState(DriveState.DEFAULT);
    shooter.setWantedState(ShooterState.SHOOT_BASIC);
    feeder.setWantedState(FeederState.SHOOT);
  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
    feeder.setWantedState(FeederState.IDLE); // Run hopper and linearizer
    intake.setWantedState(IntakeState.IDLE);
    shooter.setWantedState(ShooterState.DEFAULT);
  }

  public void handleIntakeingState() {
    intake.setWantedState(IntakeState.INTAKE);
    feeder.setWantedState(FeederState.INTAKE); // Run hopper and linearizer
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
    shooter.setWantedState(ShooterState.ZERO);
  }

  public void PARTY() {
    lights.PARTY();
  }

  @Override
  public void periodic() {
    PARTY();
    currentSuperState = handleStateTransitions();
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
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    Logger.recordOutput("Super State", currentSuperState);
    applyStates();

  }
}
