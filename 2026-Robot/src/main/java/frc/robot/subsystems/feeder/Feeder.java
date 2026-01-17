// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIOComp;
import frc.robot.subsystems.feeder.FeederIOSim;

public class Feeder extends SubsystemBase {

  private final FeederIO io;

  public Feeder() {
    if (RobotBase.isReal()) {
      io = new FeederIOComp();
    } else {
      io = new FeederIOSim();
    }
  }

  public void init() {
    io.init();
  }

  public void setFirstFeederPercent(double percent) {
    io.setFirstFeederPercent(percent);
  }

  public void setSecondFeederPercent(double percent) {
    io.setSecondFeederPercent(percent);
  }

  public enum FeederState {
    IDLE,
    INTAKE,
    OUTAKE,
    SHOOT,
  }

  public void setWantedState(FeederState wantedState) {
    this.wantedState = wantedState;
  }

  private FeederState wantedState = FeederState.IDLE;
  private FeederState systemState = FeederState.IDLE;

  private FeederState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return FeederState.IDLE;
      case INTAKE:
        return FeederState.INTAKE;
      case OUTAKE:
        return FeederState.OUTAKE;
      case SHOOT:
        return FeederState.SHOOT;
      default:
        return FeederState.IDLE;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Feeder State", systemState);
    switch (systemState) {
      case IDLE:
        setFirstFeederPercent(0.0);
        setSecondFeederPercent(0.0);
        break;
      case INTAKE:
        setFirstFeederPercent(0.5);
        setSecondFeederPercent(0.5);
        break;
      case OUTAKE:
        setFirstFeederPercent(-0.5);
        setSecondFeederPercent(-0.5);
        break;
      case SHOOT:
        setFirstFeederPercent(0.5);
        setSecondFeederPercent(0.5);
        break;
      default:
        setFirstFeederPercent(0.0);
        setSecondFeederPercent(0.0);
        break;
    }
  }
}
