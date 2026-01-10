// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final DigitalInput climbSensor = new DigitalInput(1);
  private final ClimberIO io;

  public int timesTriggered = 0;

  /** Creates a new Climiber. */
  public Climber() {
    io = RobotBase.isReal() ? new ClimberIOComp() : new ClimberIOSim();
  }

  public void init() {
    io.init();
  }

  public double getPosition() {
    return io.getPosition();
  }

  public enum ClimbState {
    EXTENDING,
    RETRACTING,
    IDLE,
    DEFAULT,
  }

  private ClimbState wantedState = ClimbState.DEFAULT;
  private ClimbState systemState = ClimbState.DEFAULT;

  private ClimbState handleStateTransition() {
    switch (wantedState) {
      case EXTENDING:
        return ClimbState.EXTENDING;
      case RETRACTING:
        return ClimbState.RETRACTING;
      case IDLE:
        return ClimbState.IDLE;
      default:
        return ClimbState.DEFAULT;
    }
  }

  public void setPivotTorque(double current, double maxPercent) {
    io.setTorque(current, maxPercent);
  }

  public void setWantedState(ClimbState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean getClimbSensor() {
    return climbSensor.get();
  }

  public boolean getTimesTriggered() {
    if (!climbSensor.get()) {
      timesTriggered += 1;
    }

    return timesTriggered >= 3;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climb Sensor", getClimbSensor());
    Logger.recordOutput("Climber Times Triggered", timesTriggered);
    ClimbState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    io.updateInputs(systemState);

    Logger.recordOutput("Climber State", systemState);
    switch (systemState) {
      case EXTENDING:
        setPivotTorque(-160.0, 1.0);
        break;
      case RETRACTING:
        setPivotTorque(160.0, 1.0);
        break;
      case IDLE:
        setPivotTorque(0, 0);
        break;
      default:
        // setPivotTorque(0, 0);
        break;
    }
  }
}
