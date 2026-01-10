// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederPickup extends Command {
  /** Creates a new FeederPickup. */
  Superstructure superstructure;

  public FeederPickup(Superstructure superstructure) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.superstructure = superstructure;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    superstructure.setWantedState(SuperState.FEEDER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (DriverStation.isAutonomousEnabled()) {
      superstructure.setWantedState(SuperState.IDLE);
    } else {
      superstructure.setWantedState(SuperState.DEFAULT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return superstructure.hasCoralSticky();
  }
}
