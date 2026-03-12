// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ZeroTurretMidMatch extends Command {
  private Shooter shooter;

  public ZeroTurretMidMatch(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
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