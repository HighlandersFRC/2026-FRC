package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Superstructure.SuperState;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private double dynamicIntakeSpeed;
  private boolean jiggleUp = true;
  private double timeZeroed = Timer.getFPGATimestamp();
  private boolean firstTimeZeroed = true;

  public Intake() {
    if (RobotBase.isReal()) {
      io = new IntakeIOComp();
    } else {
      io = new IntakeIOSim();
    }
    setIntakePosition(0.0);
  }

  public void teleopInit() {
    setWantedState(IntakeState.ZERO);
  }

  public double getIntakePosition() {
    return io.getIntakePosition();
  }

  public void setIntakePosition(double intakePosition) {
    io.setIntakePosition(intakePosition);
  }

  public void setRollerPercent(double percent) {
    io.setRollerPercent(percent);
  }

  public void setRollerTorque(double amps, double maxPercent) {
    io.setRollerTorque(amps, maxPercent);
  }

  public void setPivotTorque(double amps, double maxPercent) {
    io.setPivotTorque(amps, maxPercent);
  }

  public enum IntakeState {
    INTAKING,
    DYNAMIC_INTAKING,
    UP,
    DOWN,
    JIGGLE,
    ZERO,
    IDLE
  }

  public LoggedMechanismLigament2d getLigament() {
    return new LoggedMechanismLigament2d("Intake", Units.inchesToMeters(29), io.getIntakePosition() * 360);
  }

  private IntakeState wantedState = IntakeState.UP;
  private IntakeState systemState = IntakeState.UP;

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedState(IntakeState wantedState, ChassisSpeeds robotSpeed) {
    this.wantedState = wantedState;
    this.dynamicIntakeSpeed = 5.0 * Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
        / Constants.Physical.TOP_SPEED + 0.2;
    if (this.dynamicIntakeSpeed < 0.4) {
      this.dynamicIntakeSpeed = 0.4;
    } else if (this.dynamicIntakeSpeed > 1.0) {
      this.dynamicIntakeSpeed = 1.0;
    }
  }

  private IntakeState handleStateTransition() {
    if (OI.driverRT.getAsBoolean()) {
      return IntakeState.DYNAMIC_INTAKING;
    }
    switch (wantedState) {
      case UP:
        return IntakeState.UP;
      case DOWN:
        return IntakeState.DOWN;
      case INTAKING:
        return IntakeState.INTAKING;
      case DYNAMIC_INTAKING:
        return IntakeState.DYNAMIC_INTAKING;
      case JIGGLE:
        if (OI.driverRT.getAsBoolean()) {
          return IntakeState.DYNAMIC_INTAKING;
        } else {
          return IntakeState.JIGGLE;
        }
      case ZERO:
        if (isZeroed()) {
          wantedState = IntakeState.DOWN;
          return IntakeState.DOWN;
        }
        return IntakeState.ZERO;
      default:
        return IntakeState.IDLE;
    }
  }

  public void setIntakeUp() {
    if (getIntakePosition() < Constants.SetPoints.Intake.INTAKE_UP_POSITION + 2.0) {
      setPivotTorque(-5, 0.3);
    } else {
      setPivotTorque(-60, 1.0);
    }
  }

  public void setIntakeDown() {
    if (getIntakePosition() > Constants.SetPoints.Intake.INTAKE_DOWN_POSITION - 10.0) {
      setPivotTorque(5, 0.3);
    } else {
      setPivotTorque(40, 1.0);
    }
  }

  public void zero() {
    setPivotTorque(40, 0.5);
  }

  public boolean isZeroed() {
    if (Math.abs(io.getIntakeCurrent()) > 15.0 && io.getIntakeVelocity() < 1.0 && io.getIntakeAcceleration() < 1.0) {
      if (firstTimeZeroed) {
        timeZeroed = Timer.getFPGATimestamp();
        firstTimeZeroed = false;
      }
      if (Timer.getFPGATimestamp() - timeZeroed > 0.5) {
        io.zeroIntakePosition();
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public void setJiggle() {
    // if (getIntakePosition() > Constants.SetPoints.Intake.INTAKE_DOWN_POSITION -
    // 5.0) {
    // jiggleUp = true;
    // } else if (getIntakePosition() <
    // Constants.SetPoints.Intake.INTAKE_UP_POSITION + 15.0) {
    // jiggleUp = false;
    // }

    // if (jiggleUp) {
    // setPivotTorque(-50, 0.41);
    // } else {
    // setPivotTorque(30, 0.41);
    // }

    if (getIntakePosition() < Constants.SetPoints.Intake.INTAKE_SHOOT_POSITION) {
      setPivotTorque(-5, 0.1);
    } else {
      setPivotTorque(-30, 0.6);
    }

  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    if (systemState != IntakeState.ZERO) {
      firstTimeZeroed = true;
    }
    Logger.recordOutput("Intake/Intake State", systemState);
    Logger.recordOutput("States/Intake State", systemState);
    Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    Logger.recordOutput("Intake/Intake Position", getIntakePosition());

    Logger.recordOutput("Intake/Intake Velocity", io.getIntakeVelocity());
    Logger.recordOutput("Intake/Intake Torque", io.getIntakeCurrent());
    Logger.recordOutput("Intake/Intake Acceleration", io.getIntakeAcceleration());
    Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    switch (systemState) {
      case UP:
        setIntakeUp();
        setRollerPercent(0.0);
        break;
      case DOWN:
        setIntakeDown();
        setRollerPercent(0.0);
        break;
      case INTAKING:
        setIntakeDown();
        setRollerPercent(0.50);
        break;
      case DYNAMIC_INTAKING:
        setIntakeDown();
        setRollerTorque(80, dynamicIntakeSpeed);
        break;
      case JIGGLE:
        setJiggle();
        setRollerTorque(80, dynamicIntakeSpeed);
        break;
      case ZERO:
        setRollerPercent(0.0);
        zero();
        break;
      default:
        setIntakeUp();
        setRollerPercent(0.0);
        break;
    }
  }
}
