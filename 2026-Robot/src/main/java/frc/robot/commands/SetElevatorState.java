// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorState extends Command {
  /** Creates a new L3Setup. */
  Superstructure superstructure;
  Elevator elevator;
  SuperState superState;
  Boolean auto;

  public SetElevatorState(Superstructure superstructure, Elevator elevator, SuperState superState, Boolean auto) {
    this.superstructure = superstructure;
    this.elevator = elevator;
    this.superState = superState;
    this.auto = auto;
    addRequirements(this.superstructure);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    superstructure.setWantedState(superState);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    superstructure.setWantedState(superState);
    java.util.logging.Logger.getGlobal().finer("Elevator State Updating: " + superState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (auto && Math.abs(Constants.SetPoints.ELEVATOR_L2_POSITION_M - elevator.getElevatorPosition()) < 0.1) {
      return true;
    }
    return false;
  }
}
