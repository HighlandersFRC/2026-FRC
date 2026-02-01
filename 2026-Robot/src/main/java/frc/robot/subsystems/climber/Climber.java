// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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

    switch (systemState) {
      case CLIMBING:
        io.setPower(70, 0.5);
        break;
      case EXTEND:
        io.setPower(-70, 0.3);
        break;
      case IDLE:
        io.stop();
        break;
      default:
        break;
    }
  }
}
