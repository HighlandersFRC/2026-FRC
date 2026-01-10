// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Manipulator extends SubsystemBase {
  /** Creates a new Intake. */
  private final ManipulatorIO io;

  private boolean algaeMode = false;
  private ArmItem armItem = ArmItem.NONE;

  public void updateAlgaeMode(boolean algaeMode) {
    this.algaeMode = algaeMode;
  }

  public enum ArmItem {
    CORAL,
    ALGAE,
    NONE,
  }

  public void init() {
    io.init();
  }

  public enum ManipulatorState {
    CORAL_INTAKE,
    ALGAE_INTAKE,
    OUTAKE,
    DEFAULT,
    OFF,
  }

  private ManipulatorState wantedState = ManipulatorState.DEFAULT;
  private ManipulatorState systemState = ManipulatorState.DEFAULT;

  public void setIntakeTorque(double current, double maxPercent) {
    io.setTorque(current, maxPercent);
  }

  private boolean firstTimeCoral = true;
  private double coralTime = Timer.getFPGATimestamp();
  private boolean lastCoralValue = false;
  private double switchTime = Timer.getFPGATimestamp();
  private boolean hasCoralSticky = false;

  public boolean hasCoral() {
    if (Math.abs(io.getVelocity()) < 5.0
        && Math.abs(io.getTorqueCurrent()) > 5.0) {
      if (firstTimeCoral) {
        firstTimeCoral = false;
        coralTime = Timer.getFPGATimestamp();
      }
      if (lastCoralValue != true) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Intake Item: Has Coral");
      }
      lastCoralValue = true;
      return true;
    } else {
      firstTimeCoral = true;
      coralTime = Timer.getFPGATimestamp();
      if (lastCoralValue != false) {
        switchTime = Timer.getFPGATimestamp();
        java.util.logging.Logger.getGlobal().finer("Switch Intake Item: Empty");
      }
      lastCoralValue = false;
      return false;
    }
  }

  public boolean hasCoralSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.3) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

  public boolean hasCoralSemiSticky() {
    if (hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
      hasCoralSticky = true;
    } else if (!hasCoral() && Timer.getFPGATimestamp() - switchTime > 0.1) {
      hasCoralSticky = false;
    }
    return hasCoralSticky;
  }

  public boolean hasCoralForTime(double time) {
    if (hasCoral() && Timer.getFPGATimestamp() - coralTime > time) {
      return true;
    } else {
      return false;
    }
  }

  public ArmItem getArmItem() {
    if (!algaeMode && io.getTorqueCurrent() > 5.0) {
      if (Math.abs(io.getVelocity()) < 5.0) {
        if (Math.abs(io.getAcceleration()) < 10.0) {
          return ArmItem.CORAL;
        } else {
          return ArmItem.NONE;
        }
      } else {
        return ArmItem.NONE;
      }
    } else if (algaeMode && io.getTorqueCurrent() > 1.0) {
      if (Math.abs(io.getVelocity()) < 15.0) {
        if (true) {
          return ArmItem.ALGAE;
        }
      } else {
        return ArmItem.NONE;
      }
    }
    return ArmItem.NONE;
  }

  public Manipulator() {
    // if (RobotBase.isReal()) {
    io = new ManipulatorIOComp();
    // } else {
    // io = new ManipulatorIOSim();
    // }
  }

  public void setIntakePercent(double percent) {
    io.setPercent(percent);
  }

  public double getIntakeRPS() {
    return io.getVelocity();
  }

  public boolean inL1State = false;
  double initOutakeL1Position = 0.0;
  boolean initOutakeL1 = false;

  private ManipulatorState handleStateTransition() {
    Logger.recordOutput("povup presses", OI.driverPOVUp.getAsBoolean());
    if (!OI.driverPOVUp.getAsBoolean() || !OI.driverLT.getAsBoolean()) {
      inL1State = false;
      initOutakeL1Position = 0.0;
      initOutakeL1 = false;
    }
    if (OI.driverLT.getAsBoolean()) {
      if (OI.driverPOVUp.getAsBoolean()) {
        if (!initOutakeL1) {
          initOutakeL1Position = io.getPosition() / 12.5;
          initOutakeL1 = true;
        }

        if (Math.abs(
            Math.abs(io.getPosition()) / 12.5
                - Math.abs(initOutakeL1Position)) < 0.5) {
          return ManipulatorState.OUTAKE;
        } else {
          return ManipulatorState.OFF;
        }
      } else {
        initOutakeL1 = false;
        return ManipulatorState.OUTAKE;
      }
    }
    switch (wantedState) {
      case CORAL_INTAKE:
        return ManipulatorState.CORAL_INTAKE;
      case ALGAE_INTAKE:
        return ManipulatorState.ALGAE_INTAKE;
      case OUTAKE:
        return ManipulatorState.OUTAKE;
      case OFF:
        return ManipulatorState.OFF;
      default:
        return ManipulatorState.DEFAULT;
    }
  }

  public void setWantedState(ManipulatorState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    Logger.recordOutput("Manipulator Motor Current", io.getTorqueCurrent());
    Logger.recordOutput("Manipulator Torque Current", io.getStatorCurrent());
    Logger.recordOutput("Manipulator Velocity", io.getVelocity());
    if (armItem != getArmItem()) {
      armItem = getArmItem();
    }
    systemState = handleStateTransition();
    Logger.recordOutput("Intake State", systemState);
    Logger.recordOutput("Manipulator Has coral", hasCoral());
    Logger.recordOutput("Intake Item", armItem);
    switch (systemState) {
      case CORAL_INTAKE:
        switch (armItem) {
          case CORAL:
            setIntakeTorque(30, 1.0);
            break;
          default:
            setIntakePercent(1.0);
            break;
        }
        break;
      case ALGAE_INTAKE:
        switch (armItem) {
          case ALGAE:
            setIntakeTorque(55, 0.8);
            break;
          default:
            setIntakeTorque(60, 0.8);
            break;
        }
        break;
      case OUTAKE:
        switch (armItem) {
          default:
            if (algaeMode) {
              if (OI.driverPOVUp.getAsBoolean()) {
                setIntakePercent(-0.4);
              } else {
                setIntakePercent(-0.35);
              }
            } else {
              if (OI.driverPOVLeft.getAsBoolean()) {
                setIntakePercent(-0.5);
              } else if (OI.driverPOVDown.getAsBoolean()) {
                setIntakePercent(-0.4);
              } else if (OI.driverPOVUp.getAsBoolean()) {
                setIntakePercent(-0.25);
              } else if (OI.driverPOVRight.getAsBoolean()) {
                setIntakePercent(-0.5);
              } else {
                setIntakePercent(-1.0);

              }
            }
            break;
        }
        break;
      case OFF:
        setIntakePercent(0.0);
        break;
      default:
        if (algaeMode) {
          setIntakeTorque(67, 0.4);
        } else {
          setIntakeTorque(20, 0.2);
        }
    }
  }
}
