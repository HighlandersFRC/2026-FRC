package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private double dynamicIntakeSpeed;
  private boolean jiggleUp = true;

  public Intake() {
    if (RobotBase.isReal()) {
      io = new IntakeIOComp();
    } else {
      io = new IntakeIOSim();
    }
    setIntakePosition(0.0);
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
        return IntakeState.JIGGLE;
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

  public void setJiggle() {
    if (getIntakePosition() > Constants.SetPoints.Intake.INTAKE_DOWN_POSITION - 15.0) {
      jiggleUp = true;
    } else if (getIntakePosition() < Constants.SetPoints.Intake.INTAKE_UP_POSITION + 10.0) {
      jiggleUp = false;
    }

    if (jiggleUp) {
      setPivotTorque(-50, 0.67);
    } else {
      setPivotTorque(30, 0.67);
    }

  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Intake/Intake State", systemState);
    Logger.recordOutput("States/Intake State", systemState);
    Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    Logger.recordOutput("Intake/Intake Position", getIntakePosition());
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
        setRollerPercent(dynamicIntakeSpeed);
        break;
      case JIGGLE:
        if (OI.driverRT.getAsBoolean()) {
          setIntakeDown();
        }
        setJiggle();
        setRollerPercent(dynamicIntakeSpeed);
        break;
      default:
        setIntakeUp();
        setRollerPercent(0.0);
        break;
    }
  }
}
