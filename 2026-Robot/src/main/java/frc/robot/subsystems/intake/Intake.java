// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOComp;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  public Intake() {
    if (RobotBase.isReal()) {
      io = new IntakeIOComp();
    } else {
      io = new IntakeIOSim();
    }
  }

  public void init() {
    io.init();
  }

  public void setIntakePercent(double percent) {
    io.setIntakePercent(percent);
  }

  public enum IntakeState {
    IDLE,
    INTAKE,
    OUTAKE,
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  private IntakeState wantedState = IntakeState.IDLE;
  private IntakeState systemState = IntakeState.IDLE;

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return IntakeState.IDLE;
      case INTAKE:
        return IntakeState.INTAKE;
      case OUTAKE:
        return IntakeState.OUTAKE;
      default:
        return IntakeState.IDLE;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Intake State", systemState);
    switch (systemState) {
      case IDLE:
        setIntakePercent(0.0);
        break;
      case INTAKE:
        setIntakePercent(0.5);
        break;
      case OUTAKE:
        setIntakePercent(-0.5);
        break;
      default:
        setIntakePercent(0.0);
        break;
    }
  }
}
