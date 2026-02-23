// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

public class SetRobotStateTimeout extends Command {
  Superstructure superstructure;
  SuperState state;
  double timeout;
  double startTime;

  /** Creates a new SetShootingState. */
  public SetRobotStateTimeout(Superstructure superstructure, SuperState state, double timeout) {
    this.superstructure = superstructure;
    this.state = state;
    this.timeout = timeout;
    addRequirements(superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    superstructure.setWantedState(state);
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
    return Timer.getFPGATimestamp() - startTime >= timeout;
  }
}
