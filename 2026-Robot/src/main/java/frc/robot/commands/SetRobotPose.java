// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRobotPose extends Command {
  /** Creates a new SetRobotPose. */
  Drive drive;
  double x;
  double y;
  double theta;

  // Translation2d translation2d;
  // Rotation2d rotation2d;
  public SetRobotPose(Drive drive, double x, double y, double theta) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.theta = theta;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d translation2d = new Translation2d(x, y);
    Rotation2d rotation2d = new Rotation2d(theta);
    Pose2d pose = new Pose2d(translation2d, rotation2d);
    drive.setOdometry(pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation2d = new Translation2d(x, y);
    Rotation2d rotation2d = new Rotation2d(theta);
    Pose2d pose = new Pose2d(translation2d, rotation2d);
    drive.setOdometry(pose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
