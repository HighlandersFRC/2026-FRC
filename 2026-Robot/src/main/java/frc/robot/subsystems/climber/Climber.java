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
    AUTON_RETRACT,
    AUTON_EXTEND,
    L3_CLIMBING,
    MANUAL_RETRACT,
    MANUAL_EXTEND
  }

  public enum L3ClimberState {
    L1EXTEND,
    L1RETRACT,
    L2EXTEND,
    L2RETRACT,
    L3EXTEND,
    L3RETRACT
  }

  public final ClimberIO io;

  private ClimberState wantedState = ClimberState.IDLE;
  private ClimberState systemState = ClimberState.IDLE;
  private L3ClimberState l3State = L3ClimberState.L1EXTEND;

  public double getClimberPosition() {
    return io.getPosition();
  }

  public Climber() {
    this.io = new ClimberIOComp();
  }

  public void stopClimber() {
    io.stop();
  }

  public void retractClimber() {
    io.setPower(-100, 0.2);
  }

  public void extendClimber() {
    io.setPower(100, 0.2);
  }

  public void stallClimber() {
    io.setPower(-100, 0.2);
  }

  public void setWantedState(ClimberState wantedState) {
    this.wantedState = wantedState;
  }

  private ClimberState handleStateTransition() {
    switch (wantedState) {
      case AUTON_RETRACT:
        return ClimberState.AUTON_RETRACT;
      case L3_CLIMBING:
        return ClimberState.L3_CLIMBING;
      case MANUAL_RETRACT:
        return ClimberState.MANUAL_RETRACT;
      case AUTON_EXTEND:
        return ClimberState.AUTON_EXTEND;
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
      case AUTON_RETRACT:
        if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_AUTON_L1_RETRACT_HEIGHT_INCHES
            + Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) { // TODO: tune for just the
          // L1 climb in auto
          stallClimber();
          Logger.recordOutput("Climber/Output", "Stopped Auton Retracting");
        } else {
          retractClimber();
          Logger.recordOutput("Climber/Output", "Auton Retracting");
        }
        break;
      case AUTON_EXTEND:
        if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L1_EXTEND_HEIGHT_INCHES
            - Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
          stopClimber();
          Logger.recordOutput("Climber/Output", "Stopped Auton Extending");
        } else {
          extendClimber();
          Logger.recordOutput("Climber/Output", "Auton Extending");
        }
        break;
      case MANUAL_EXTEND:
        extendClimber();
        Logger.recordOutput("Climber/Output", "Manual Extending");
        break;
      case MANUAL_RETRACT:
        retractClimber();
        Logger.recordOutput("Climber/Output", "Manual Retracting");
        break;
      case L3_CLIMBING:
        switch (l3State) {
          case L1EXTEND:
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L1_EXTEND_HEIGHT_INCHES
                - Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stopClimber();
              Logger.recordOutput("Climber/Output", "Stopped Extending L1");
            } else {
              extendClimber();
              Logger.recordOutput("Climber/Output", "Extending L1");
            }
            break;
          case L1RETRACT:
            if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_TELEOP_L1_RETRACT_HEIGHT_INCHES
                + Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stallClimber(); // TODO: change to stop maybe
              Logger.recordOutput("Climber/Output", "Stopped Retracting L1");
            } else {
              retractClimber();
              Logger.recordOutput("Climber/Output", "Retracting L1");
            }
          case L2EXTEND:
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L2_EXTEND_HEIGHT_INCHES
                - Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stopClimber();
              Logger.recordOutput("Climber/Output", "Stopped Extending L2");
            } else {
              extendClimber();
              Logger.recordOutput("Climber/Output", "Extending L2");
            }
            break;
          case L2RETRACT:
            if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_TELEOP_L2_RETRACT_HEIGHT_INCHES
                + Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stallClimber(); // TODO: change to stop maybe
              Logger.recordOutput("Climber/Output", "Stopped Retracting L2");
            } else {
              retractClimber();
              Logger.recordOutput("Climber/Output", "Retracting L2");
            }
          case L3EXTEND:
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L3_EXTEND_HEIGHT_INCHES
                - Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stopClimber();
              Logger.recordOutput("Climber/Output", "Stopped Extending L3");
            } else {
              extendClimber();
              Logger.recordOutput("Climber/Output", "Extending L3");
            }
            break;
          case L3RETRACT:
            if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_TELEOP_L3_RETRACT_HEIGHT_INCHES
                + Constants.SetPoints.Climber.CLIMBER_MOVEMENT_DEADZONE) {
              stallClimber(); // TODO: change to stop maybe
              Logger.recordOutput("Climber/Output", "Stopped Retracting L3");
            } else {
              retractClimber();
              Logger.recordOutput("Climber/Output", "Retracting L3");
            }
            break;
          default:
            System.out.println("uh oh");
            break;
        }
        break;
      case IDLE:
        stopClimber();
        break;
      default:
        stopClimber();
        break;
    }
  }
}
