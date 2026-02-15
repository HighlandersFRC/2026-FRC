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
    HOP, // Move balls toward shooter
    FEED, // Move balls into linearizer
    SHOOT, // Move balls into shooter
    DEFAULT, // hop but slower
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
      case HOP:
        return FeederState.HOP;
      case FEED:
        if (getLinearizerSensorTripped()) {
          return FeederState.HOP; // Only run hopper
        } else {
          return FeederState.FEED; // Run hopper and linearizer
        }
      case SHOOT:
        return FeederState.SHOOT;
      case DEFAULT:
        return FeederState.DEFAULT;
      default:
        return FeederState.IDLE;
    }
  }

  public void setHopperPercent(double percent) {
    io.setHopperPercent(percent);
  }

  public void setLinearizerPercent(double percent) {
    io.setLinearizerPercent(percent);
  }

  public void setLinearizerTorque(double amps, double maxPercent) {
    io.setLinearizerTorque(amps, maxPercent);
  }

  public void setLinearizerSpeed(double metersPerSecond) {
    io.setLinearizerSpeed(
        metersPerSecond);
  }

  public double getLinearizerSpeed() {
    return io.getLinearizerSpeed();
  }

  public void setHopperSpeed(double metersPerSecond) {
    io.setHopperSpeed(
        metersPerSecond);
  }

  public void setHopperTorque(double amps, double maxpercent) {
    io.setHopperTorque(amps, maxpercent);
  }

  public double getHopperSpeed() {
    return io.getHopperSpeed();
  }

  public boolean getLinearizerSensorTripped() {
    return io.getLinearizerSensorTripped();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    switch (systemState) {
      case HOP:
        // setHopperPercent(Constants.SetPoints.Feeder.HOPPER_PERCENT);
        setHopperTorque(40, 0.5);
        setLinearizerPercent(0.0);
        break;
      case FEED:
        // setHopperPercent(Constants.SetPoints.Feeder.HOPPER_PERCENT);
        // setLinearizerPercent(-Constants.SetPoints.Feeder.LINEARIZER_PERCENT);
        setHopperTorque(40, 0.5);
        setLinearizerTorque(-67, 0.50);
        break;
      case SHOOT:
        // setHopperPercent(Constants.SetPoints.Feeder.HOPPER_PERCENT);
        // setLinearizerPercent(-Constants.SetPoints.Feeder.LINEARIZER_PERCENT);
        setHopperTorque(40, 0.5);
        setLinearizerTorque(-67, 0.50);
        break;
      case DEFAULT:
        // setHopperPercent(0.1);
        setHopperTorque(10, 0.2);
        setLinearizerPercent(0.0);
        break;
      default:
        setHopperPercent(0.0);
        setLinearizerPercent(0.0);
        break;
    }
    Logger.recordOutput("Feeder/Feeder State", systemState);
    Logger.recordOutput("States/Feeder State", systemState);
  }
}
