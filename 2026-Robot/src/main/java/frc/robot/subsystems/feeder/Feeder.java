// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.tools.logging.TunableNumber;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public enum FeederState {
    IDLE, // Stop all movement
    FEED, // Move balls into shooter
    REVERSE, // Reverses for some reason idk maybe to unclog stuff
    DEFAULT, // reverses slowly to reduce clogs
  }

  private final FeederIO io;
  // private TunableNumber feederSpeed = new TunableNumber("Feeder speed", 120.0);

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
    if (OI.getOperatorLB()) {
      return FeederState.REVERSE;
    } else if (OI.getOperatorRB()) {
      return FeederState.FEED;
    }
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

  public void setDyeRotorRPM(double rpm) {
    Logger.recordOutput("Feeder/Dye Rotor RPM Setpoint", rpm);
    io.setDyeRotorRPM(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    // if (OI.driverA.getAsBoolean()) {
    // setDyeRotorRPM(120.0);
    // } else if (OI.driverB.getAsBoolean()) {
    // setDyeRotorRPM(-60.0);
    // } else if (OI.driverX.getAsBoolean()) {
    // setDyeRotorRPM(150.0);
    // } else {
    // setDyeRotorPercent(0);
    // }
    switch (systemState) {
      case FEED:
        setDyeRotorTorque(80, 0.8);
        // setDyeRotorRPM(140.0);
        // setDyeRotorPercent(1.0);
        break;
      case REVERSE:
        setDyeRotorTorque(-80, 0.4);
        // setDyeRotorPercent(-0.4);
        // setDyeRotorRPM(-60.0);
        break;
      case DEFAULT:
        // setDyeRotorTorque(-30, 0.1);
        setDyeRotorPercent(0.0);
        break;
      default:
        setDyeRotorPercent(0.0);
        break;
    }
    Logger.recordOutput("Feeder/Feeder State", systemState);
    Logger.recordOutput("States/Feeder State", systemState);
    Logger.recordOutput("Feeder/Dye Rotor Current", io.getDyeRotorCurrent());
    Logger.recordOutput("Feeder/Dye Rotor RPM", io.getDyeRotorRPM());
  }
}
