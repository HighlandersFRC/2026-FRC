package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

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

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case UP:
        return IntakeState.UP;
      case INTAKING:
        return IntakeState.INTAKING;
      default:
        return IntakeState.IDLE;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    Logger.recordOutput("Intake State", systemState);
    switch (systemState) {
      case UP:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_UP_POSITION);
        setRollerPercent(0.0);
        break;
      case INTAKING:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_DOWN_POSITION);
        setRollerPercent(0.7);
        break;
      default:
        // setIntakePosition(Constants.SetPoints.Intake.INTAKE_UP_POSITION);
        setRollerPercent(0.0);
        break;
    }
  }
}
