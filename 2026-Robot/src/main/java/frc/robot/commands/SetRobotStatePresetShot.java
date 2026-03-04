// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.tools.math.ShotCalculator.ShotSolution;

public class SetRobotStatePresetShot extends Command {
  Superstructure superstructure;
  ShotSolution shotSolution;

  /** Creates a new SetShootingState. */
  public SetRobotStatePresetShot(Superstructure superstructure, ShotSolution shotSolution) {
    this.superstructure = superstructure;
    this.shotSolution = shotSolution;
    addRequirements(superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    superstructure.setWantedState(SuperState.PRESET_SHOOT, shotSolution);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    java.util.logging.Logger.getGlobal().fine("Interrupted? " + interrupted);
    if (DriverStation.isAutonomousEnabled()) {
      superstructure.setWantedState(SuperState.IDLE);
    } else {
      superstructure.setWantedState(SuperState.DEFAULT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
