// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.tools.wrappers.AutoFollower;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCoralGroundPickupFollower extends AutoFollower {
  Superstructure superstructure;
  Drive drive;
  private int currentPathPointIndex = 0;
  private double initTime = 0.0;
  double timeout = 0.0;

  /** Creates a new AutoPlaceL4Follower. */
  public AutoCoralGroundPickupFollower(Superstructure superstructure, Drive drive, double timeout) {
    this.superstructure = superstructure;
    this.drive = drive;
    this.timeout = timeout;
    addRequirements(superstructure, drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public int getPathPointIndex() {
    return currentPathPointIndex;
  }

  public void from(int pointIndex, JSONObject pathJSON, int to) {
    java.util.logging.Logger.getGlobal().fine("Running L4 in auto");
    this.currentPathPointIndex = pointIndex;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    superstructure.setWantedState(SuperState.AUTO_GROUND_CORAL_PICKUP_FRONT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    java.util.logging.Logger.getGlobal().finer("Running Auto Pickup Follower");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    superstructure.setWantedState(SuperState.PASSOFF_IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (superstructure.intakeHasCoral() || Timer.getFPGATimestamp() - initTime > timeout) {
      return true;
    } else {
      return false;
    }
  }
}
