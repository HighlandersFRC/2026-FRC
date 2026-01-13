package frc.robot.subsystems;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightsState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.tools.math.PhysicsModel;
import frc.robot.tools.math.Vector;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Lights lights;
  private final Shooter shooter;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private Translation3d trajectoryPoint = new Translation3d(0, 0, 0);
  private Translation3d trajectoryVelocity = new Translation3d(0, 0, 0);

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public boolean algaeMode = false;

  public Superstructure(Drive drive,
      Lights lights, Shooter shooter) {
    this.drive = drive;
    this.lights = lights;
    this.shooter = shooter;
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
      default:
        handleIdleState();
        break;
    }
  }

  private void handleShootState() {
    Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
        .getMt2Pose2dY(), 0)
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
    Vector3D initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
    Vector robotVelocity = drive.getRobotVelocityVector();
    Vector3D onTheMove = new Vector3D(initialVelocity.getX() - robotVelocity.getI(),
        initialVelocity.getY() - robotVelocity.getJ(),
        initialVelocity.getZ());
    Vector3D trajectoryVector = initialVelocity;
    Translation3d trajectory = new Translation3d(
        trajectoryVector.getX(),
        trajectoryVector.getY(),
        trajectoryVector.getZ());
    Translation3d loggedTrajectory = trajectory.plus(initial);
    Logger.recordOutput("Trajectory", loggedTrajectory);
    Logger.recordOutput("Turret Position", new Pose3d(initial, new Rotation3d(drive.getMt2Pose2d().getRotation())
        .plus(new Rotation3d(shooter.getRobotRelativeTurretAngle()))));
    gyro = gyro.unaryMinus();
    trajectory = new Translation3d(
        trajectory.getX() * gyro.getCos() - trajectory.getY() * gyro.getSin(),
        trajectory.getX() * gyro.getSin() + trajectory.getY() * gyro.getCos(),
        trajectory.getZ());
    trajectoryVector = new Vector3D(trajectory.getX(), trajectory.getY(), trajectory.getZ());
    shooter.setWantedState(Shooter.ShooterState.SHOOT, trajectoryVector);
    Vector3D realVector = shooter.getCurrentShooterTrajectory();
    gyro = gyro.unaryMinus();
    Translation3d realTrajectory = new Translation3d(
        realVector.getX() * gyro.getCos() - realVector.getY() * gyro.getSin(),
        realVector.getX() * gyro.getSin() + realVector.getY() * gyro.getCos(),
        realVector.getZ());
    Logger.recordOutput("Shooter Trajectory",
        realTrajectory);
    Logger.recordOutput("Shooter State", "SHOOT");
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
        currentSuperState = SuperState.SHOOT;
        Translation3d initial = new Translation3d(drive.getMt2Pose2dX(), drive
            .getMt2Pose2dY(), 0)
            .plus(Constants.Physical.Shooter.SHOOTER_POSITION
                .rotateBy(new Rotation3d(drive.getMt2Pose2d().getRotation())));
        Translation3d target;
        if (Globals.fieldSide.equals("blue")) {
          target = Constants.Field.HUB_POSE_BLUE;
        } else {
          target = Constants.Field.HUB_POSE_RED;
        }
        double distance2D = initial.toTranslation2d().getDistance(target.toTranslation2d());
        double height = Constants.Physical.Shooter.getTrajectoryHeight(distance2D);
        Vector3D initialVelocity = PhysicsModel.getHeightBoundTrajectory(initial, target, height);
        trajectoryPoint = initial;
        trajectoryVelocity = new Translation3d(initialVelocity.getX(), initialVelocity.getY(), initialVelocity.getZ());
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;

  }

  public void handleDefaultState() {
    lights.setWantedState(LightsState.DEFAULT);
    drive.setWantedState(DriveState.DEFAULT);
  }

  public void handleIdleState() {
    drive.setWantedState(DriveState.IDLE);
    lights.setWantedState(LightsState.DEFAULT);
  }

  public void PARTY() {
    lights.PARTY();
  }

  @Override
  public void periodic() {
    PARTY();
    currentSuperState = handleStateTransitions();
    trajectoryVelocity = new Translation3d(trajectoryVelocity.getX(),
        trajectoryVelocity.getY(),
        trajectoryVelocity.getZ() - Constants.G * Globals.loopPeriodSecs);
    trajectoryPoint = trajectoryPoint.plus(trajectoryVelocity.times(Globals.loopPeriodSecs));
    Logger.recordOutput("Trajectory Point", new Pose3d(trajectoryPoint, new Rotation3d()));
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }

    applyStates();
  }
}
