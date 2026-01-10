// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;
  private IntakeState wantedState = IntakeState.DEFAULT;
  private IntakeState systemState = IntakeState.DEFAULT;
  private boolean isZeroed = false;
  private boolean firstTimeHandOff = true;

  public enum IntakeState {
    INTAKING,
    OUTAKING,
    HANDOFF,
    IDLE,
    DEFAULT,
    ZERO,
    DOWN,
  }

  public Intake() {
    // if (RobotBase.isReal()) {
    io = new IntakeIOComp();
    // } else {
    // io = new IntakeIOSim();
    // }
  }

  public void init() {
    io.init();
  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case INTAKING:
        return IntakeState.INTAKING;
      case ZERO:
        return IntakeState.ZERO;
      case OUTAKING:
        return IntakeState.OUTAKING;
      case IDLE:
        return IntakeState.IDLE;
      case DEFAULT:
        return IntakeState.DEFAULT;
      case HANDOFF:
        return IntakeState.HANDOFF;
      case DOWN:
        return IntakeState.DOWN;
      default:
        return IntakeState.IDLE;
    }
  }

  public void pivotWithTorque(double current, double maxPercent) {
    io.setPivotTorque(current, maxPercent);
  }

  public void pivotToPosition(double pivotRotations) {
    io.setPivotPosition(pivotRotations);
  }

  public void setRollerCurrent(double amps, double maxPercent) {
    io.setRollerCurrent(amps, maxPercent);
  }

  public void setRollerPercent(double percent) {
    io.setRollerPercent(percent);
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return io.getPivotPosition();
  }

  public boolean getZeroed() {
    if (Math.abs(io.getPivotStatorCurrent()) > 10.0
        && Math.abs(io.getPivotVelocity()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  private boolean firstTimeDefault = true;
  private double defaultTime = 0.0;

  @Override
  public void periodic() {
    Logger.recordOutput("Intake Position", getPosition() * 360);
    Logger.recordOutput("Intake Motor Current",
        io.getRollerStatorCurrent());
    Logger.recordOutput("Intake Speed", io.getRollerVelocity());
    Logger.recordOutput("Intake Current", io.getRollerStatorCurrent());
    systemState = handleStateTransition();
    if (systemState != IntakeState.HANDOFF) {
      firstTimeHandOff = true;
    }
    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Intake Has coral", hasCoral());
    if (systemState != IntakeState.DEFAULT) {
      firstTimeDefault = true;
      defaultTime = Timer.getFPGATimestamp();
    }
    switch (systemState) {
      case INTAKING:
        if (Math.abs(getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN) < 25 / 360.0) {
          pivotWithTorque(20, 0.3);
        } else {
          pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        }
        setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);
        break;
      case DOWN:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        setRollerCurrent(0.0,
            0.0);
        break;
      case ZERO:
        setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_HOLDING_SPEED);
        pivotWithTorque(-40, 0.5);
        if (getZeroed()) {
          io.setPivotEncoderPosition(0.0);
        }
        break;
      case OUTAKING:
        //
        setRollerCurrent(-80,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);
        setRollerPercent(-1.0);
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_DOWN);
        break;
      case HANDOFF:
        if (firstTimeHandOff) {
          firstTimeHandOff = false;
        }
        if (Math.abs(getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 10.0 / 360.0) {
          pivotWithTorque(-5, 0.2);
        } else if (Math.abs(getPosition() -
            Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 30.0 / 360.0) {
          pivotWithTorque(-30, 0.1);
        } else if (Math.abs(getPosition() -
            Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 60.0 / 360.0) {
          pivotWithTorque(-40, 0.1);
        } else {
          pivotWithTorque(-60, 0.6);

        }

        setRollerCurrent(-80,
            Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_MAX_SPEED);

        break;
      case IDLE:
        pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
        io.setRollerPercent(0.0);
        break;
      case DEFAULT:
        if (firstTimeDefault) {
          firstTimeDefault = false;
          defaultTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - defaultTime > 1.0) {
          setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_HOLDING_TORQUE,
              0.2);
        } else {
          setRollerCurrent(Constants.SetPoints.IntakeSetpoints.INTAKE_ROLLER_TORQUE,
              0.5);
        }
        if (Math.abs(getPosition() - Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 10.0 / 360.0) {
          pivotWithTorque(-5, 0.2);
        } else if (Math.abs(getPosition() -
            Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 30.0 / 360.0) {
          pivotWithTorque(-30, 0.1);
        } else if (Math.abs(getPosition() -
            Constants.SetPoints.IntakeSetpoints.INTAKE_UP) < 60.0 / 360.0) {
          pivotWithTorque(-40, 0.1);
          // System.out.println("40");
        } else {
          // pivotToPosition(Constants.SetPoints.IntakeSetpoints.INTAKE_UP);
          pivotWithTorque(-60, 0.6);
          // System.out.println("70");

        }
        break;
      default:
        if (Math.abs(io.getPivotVelocity()) < 0.01 && !isZeroed) {
          this.setPivotCurrent(-10, 0.1);
          io.setPivotEncoderPosition(0.0);
          ;
        } else if (!isZeroed) {
          this.setPivotCurrent(-40, 0.3);
        } else {
          this.setPivotCurrent(-5, 0.1);
        }
        if (Math.abs(io.getPivotPosition()) > 2) {
          isZeroed = false;
        }
    }
  }

  private void setPivotCurrent(double amps, double maxPercent) {
    io.setPivotCurrent(amps, maxPercent);
  }

  private boolean lastCoralValue = false;
  private double switchTime = Timer.getFPGATimestamp();
  private boolean hasCoralSticky = false;

  public boolean hasCoral() {
    if (Math.abs(io.getRollerVelocity()) < 5
        && Math.abs(io.getRollerTorqueCurrent()) > 20) {
      if (lastCoralValue != true) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Ground Intake Item: Has Coral");
      }
      lastCoralValue = true;
      return true;
    } else {
      if (lastCoralValue != false) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Ground Intake Item: Empty");
      }
      lastCoralValue = false;
      return false;
    }
  }

  public boolean hasCoralSuperSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.05) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 1.5) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

}
