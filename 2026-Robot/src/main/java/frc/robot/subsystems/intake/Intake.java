package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private double dynamicIntakeSpeed;

  public Intake() {
    if (RobotBase.isReal()) {
      io = new IntakeIOComp();
    } else {
      io = new IntakeIOSim();
    }
  }

  public void init() {
    io.init();
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

  public enum IntakeState {
    INTAKING,
    DYNAMIC_INTAKING,
    UP,
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
      case INTAKING:
        return IntakeState.INTAKING;
      case DYNAMIC_INTAKING:
        return IntakeState.DYNAMIC_INTAKING;
      default:
        return IntakeState.IDLE;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Intake/Intake State", systemState);
    Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    Logger.recordOutput("States/Intake State", systemState);
    switch (systemState) {
      case UP:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_UP_POSITION);
        setRollerPercent(0.0);
        break;
      case INTAKING:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_DOWN_POSITION);
        setRollerPercent(0.50);
        break;
      case DYNAMIC_INTAKING:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_DOWN_POSITION);
        setRollerPercent(dynamicIntakeSpeed);
        break;
      default:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_UP_POSITION);
        setRollerPercent(0.0);
        break;
    }
  }
}
