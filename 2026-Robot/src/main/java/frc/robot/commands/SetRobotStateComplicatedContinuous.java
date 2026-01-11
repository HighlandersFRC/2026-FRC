// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

public class SetRobotStateComplicatedContinuous extends Command {
    Superstructure superstructure;
    SuperState startState;
    SuperState endState;

    /** Creates a new SetShootingState. */
    public SetRobotStateComplicatedContinuous(Superstructure superstructure, SuperState startState,
            SuperState endState) {
        this.superstructure = superstructure;
        this.startState = startState;
        this.endState = endState;
        addRequirements(superstructure);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        superstructure.setWantedState(startState);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        superstructure.setWantedState(endState);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
