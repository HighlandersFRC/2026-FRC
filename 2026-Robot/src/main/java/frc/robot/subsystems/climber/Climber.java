// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeState;
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
    L2EXTEND,
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
    io.setPower(-100, 0.7);
  }

  public void extendClimber() {
    io.setPower(100, 0.7);
  }

  public void stallClimber() {
    io.setPower(-100, 0.7);
  }

  public void setWantedState(ClimberState wantedState) {
    this.wantedState = wantedState;
  }

  private ClimberState handleStateTransition() {
    switch (wantedState) {
      case L3_CLIMBING:
        return ClimberState.L3_CLIMBING;
      case MANUAL_RETRACT:
        return ClimberState.MANUAL_RETRACT;
      case MANUAL_EXTEND:
        return ClimberState.MANUAL_EXTEND;
      case AUTON_EXTEND:
        return ClimberState.AUTON_EXTEND;
      case AUTON_RETRACT:
        return ClimberState.AUTON_RETRACT;
      default:
        return ClimberState.IDLE;
    }
  }

  @Override
  public void periodic() {
    if (systemState != handleStateTransition() && handleStateTransition() == ClimberState.L3_CLIMBING) {
      l3State = L3ClimberState.L1EXTEND;
    }
    systemState = handleStateTransition();
    Logger.recordOutput("Climber/Climber State", systemState);
    Logger.recordOutput("States/Climber State", systemState);
    Logger.recordOutput("Climber/Climber Position", getClimberPosition());
    Logger.recordOutput("Climber/Climber Slave Current", io.getSlaveCurrent());
    Logger.recordOutput("Climber/Climber Master Current", io.getMasterCurrent());
    switch (systemState) {
      case AUTON_RETRACT: // L1 retract in auto
        if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_AUTON_L1_RETRACT_HEIGHT_INCHES) {
          stopClimber();
          Logger.recordOutput("Climber/Output", "Stopped Auton Retracting");
        } else {
          retractClimber();
          Logger.recordOutput("Climber/Output", "Auton Retracting");
        }
        break;
      case AUTON_EXTEND: // L1 extend in auto
        if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L1_EXTEND_HEIGHT_INCHES) {
          stopClimber();
          Logger.recordOutput("Climber/Output", "Stopped Auton Extending");
        } else {
          extendClimber();
          Logger.recordOutput("Climber/Output", "Auton Extending");
        }
        break;
      case MANUAL_EXTEND: // manual mode, no restrictions
        extendClimber();
        Logger.recordOutput("Climber/Output", "Manual Extending");
        break;
      case MANUAL_RETRACT: // manual mode, no restrictions
        retractClimber();
        Logger.recordOutput("Climber/Output", "Manual Retracting");
        break;
      case L3_CLIMBING: // auto l3 climb state
        switch (l3State) {
          case L1EXTEND: // step 1
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L1_EXTEND_HEIGHT_INCHES) {
              l3State = L3ClimberState.L2EXTEND;
              Logger.recordOutput("Climber/Output", "Stopped Extending L1");
            } else {
              extendClimber();
              Logger.recordOutput("Climber/Output", "Extending L1");
            }
            break;
          case L2EXTEND: // step 2
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L2_EXTEND_HEIGHT_INCHES) {
              l3State = L3ClimberState.L3EXTEND;
              Logger.recordOutput("Climber/Output", "Stopped Extending L2");
            } else {
              retractClimber();
              Logger.recordOutput("Climber/Output", "Extending L2");
            }
            break;
          case L3EXTEND: // step 3
            if (getClimberPosition() > Constants.SetPoints.Climber.CLIMBER_L3_EXTEND_HEIGHT_INCHES) {
              l3State = L3ClimberState.L3RETRACT;
              Logger.recordOutput("Climber/Output", "Stopped Extending L3");
            } else {
              extendClimber();
              Logger.recordOutput("Climber/Output", "Extending L3");
            }
            break;
          case L3RETRACT: // step 4 (final step)
            if (getClimberPosition() < Constants.SetPoints.Climber.CLIMBER_L3_RETRACT_HEIGHT_INCHES) {
              stopClimber();
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
