// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public enum FeederState {
    IDLE, // Stop all movement
    FEED, // Move balls into shooter
    REVERSE, // Reverses for some reason idk maybe to unclog stuff
    DEFAULT, // reverses slowly to reduce clogs
  }

  private final FeederIO io;

  private FeederState wantedState = FeederState.IDLE;
  private FeederState systemState = FeederState.IDLE;

  public Feeder() {
    if (RobotBase.isReal()) {
      this.io = new FeederIOComp();
    } else {
      this.io = new FeederIOSim();
    }
  }

  public void setWantedState(FeederState wantedState) {
    this.wantedState = wantedState;
  }

  private FeederState handleStateTransition() {
    switch (wantedState) {
      case FEED:
        return FeederState.FEED;
      case REVERSE:
        return FeederState.REVERSE;
      case DEFAULT:
        return FeederState.DEFAULT;
      default:
        return FeederState.IDLE;
    }
  }

  public void setDyeRotorPercent(double percent) {
    io.setDyeRotorPercent(percent);
  }

  public void setDyeRotorTorque(double amps, double maxpercent) {
    io.setDyeRotorTorque(amps, maxpercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    switch (systemState) {
      case FEED:
        setDyeRotorTorque(80, 0.7);
        break;
      case REVERSE:
        setDyeRotorTorque(-80, 0.7);
        break;
      case DEFAULT:
        setDyeRotorTorque(-20, 0.3);
        break;
      default:
        setDyeRotorPercent(0.0);
        break;
    }
    Logger.recordOutput("Feeder/Feeder State", systemState);
    Logger.recordOutput("States/Feeder State", systemState);
  }
}
