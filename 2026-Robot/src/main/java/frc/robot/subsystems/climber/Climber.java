// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  public enum ClimberState {
    IDLE,
    CLIMBING,
    EXTEND
  }

  public final ClimberIO io;

  private ClimberState wantedState = ClimberState.IDLE;
  private ClimberState systemState = ClimberState.IDLE;

  public double getClimberPosition() {
    return io.getPosition();
  }

  public Climber() {
    this.io = new ClimberIOComp();
  }

  public void setWantedState(ClimberState wantedState) {
    this.wantedState = wantedState;
  }

  private ClimberState handleStateTransition() {
    switch (wantedState) {
      case CLIMBING:
        return ClimberState.CLIMBING;
      case EXTEND:
        return ClimberState.EXTEND;
      default:
        return ClimberState.IDLE;
    }
  }

  @Override
  public void periodic() {
    systemState = handleStateTransition();
    Logger.recordOutput("Climber/Climber State", systemState);
    Logger.recordOutput("States/Climber State", systemState);
    Logger.recordOutput("Climber/Climber Position", getClimberPosition());
    switch (systemState) {
      case CLIMBING:
        if (getClimberPosition() < 3.0) {
          io.stop();
          Logger.recordOutput("Climber/Output", "Stopped Climbing");
        } else {
          io.setPower(-70, 0.5);
          Logger.recordOutput("Climber/Output", "Climbing");
        }
        break;
      case EXTEND:
        if (getClimberPosition() > Constants.Ratios.Climber.CLIMBER_MAX_ROTATIONS - 3.0) {
          io.stop();
          Logger.recordOutput("Climber/Output", "Stopped Extending");
        } else {
          io.setPower(70, 0.3);
          Logger.recordOutput("Climber/Output", "Extending");
        }
        break;
      case IDLE:
        io.stop();
        break;
      default:
        io.stop();
        break;
    }
  }
}
