// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ZeroAngleMidMatch extends Command {
  private Drive drive;
  private Shooter shooter;

  public ZeroAngleMidMatch(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drive.zeroIMU();
    shooter.zeroTurretToEncoder();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}