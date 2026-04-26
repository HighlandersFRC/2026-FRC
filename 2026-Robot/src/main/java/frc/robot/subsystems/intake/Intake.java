package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private boolean jiggleUp = true;
  private double timeZeroed = Timer.getFPGATimestamp();
  private boolean firstTimeZeroed = true;
  private double flipJiggleTime = Timer.getFPGATimestamp();

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
    OUTAKE,
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

  private IntakeState handleStateTransition() {
    if (OI.driverRT.getAsBoolean()) {
      return IntakeState.DYNAMIC_INTAKING;
    }
    if (OI.driverRB.getAsBoolean()) {
      return IntakeState.OUTAKE;
    }
    switch (wantedState) {
      case UP:
        return IntakeState.UP;
      case DOWN:
        return IntakeState.DOWN;
      case INTAKING:
        return IntakeState.INTAKING;
      case OUTAKE:
        return IntakeState.OUTAKE;
      case DYNAMIC_INTAKING:
        if (!OI.driverRT.getAsBoolean() && DriverStation.isTeleopEnabled()) {
          return IntakeState.DOWN;
        }
        return IntakeState.DYNAMIC_INTAKING;
      case JIGGLE:
        if (OI.driverRT.getAsBoolean() && DriverStation.isTeleopEnabled()) {
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
      setPivotTorque(-40, 1.0);
    }
  }

  public void setIntakeDown() {
    if (getIntakePosition() > Constants.SetPoints.Intake.INTAKE_DOWN_POSITION - 10.0) {
      if (DriverStation.isAutonomousEnabled()) {
        setPivotTorque(40, 1.0);
      } else {
        setPivotTorque(5, 0.3);
      }
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
    // 5.002) {
    // jiggleUp = true;
    // } else if (getIntakePosition() <
    // Constants.SetPoints.Intake.INTAKE_UP_POSITION + 17.18) {
    // jiggleUp = false;
    // }
    if (jiggleUp) {

      if (getIntakePosition() < Constants.SetPoints.Intake.INTAKE_UP_POSITION + 10.50) {
        setPivotTorque(30, 0.3);
      } else {
        setPivotTorque(-45, 0.6);
      }
    } else {
      setPivotTorque(30, 0.5);
    }
  }

  public void calcJiggle() {
    if (Timer.getFPGATimestamp() - flipJiggleTime > 0.3) {
      // if (getIntakePosition() < Constants.SetPoints.Intake.INTAKE_UP_POSITION +
      // 15.50) {
      // jiggleUp = false;
      // } else {
      jiggleUp = !jiggleUp;
      // }
      flipJiggleTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    calcJiggle();
    systemState = handleStateTransition();
    if (systemState != IntakeState.ZERO) {
      firstTimeZeroed = true;
    }
    // Logger.recordOutput("Intake/Intake State", systemState);
    Logger.recordOutput("States/Intake State", systemState);
    // Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    Logger.recordOutput("Intake/Intake Position", getIntakePosition());

    Logger.recordOutput("Intake/Intake Velocity", io.getIntakeVelocity());
    Logger.recordOutput("Intake/Intake Pivot Current", io.getIntakeCurrent());
    Logger.recordOutput("Intake/Intake Roller Current",
        io.getIntakeRollerCurrent());
    // Logger.recordOutput("Intake/Intake Acceleration",
    // io.getIntakeAcceleration());
    // Logger.recordOutput("Intake/Dynamic Intake Speed", dynamicIntakeSpeed);
    Logger.recordOutput("Intake Roller Vel", io.getIntakeRollerVelocity());
    Logger.recordOutput("Intake Follower Roller Vel",
        io.getIntakeFollowerRollerVelocity());
    // Logger.recordOutput("Intake Roller Temp", io.getIntakeRollerTemp());
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
        // setRollerPercent(0.9);
        setRollerTorque(80, 0.75);

        break;
      case OUTAKE:
        setIntakeDown();
        setRollerPercent(-0.9);
        break;
      case DYNAMIC_INTAKING:
        setIntakeDown();
        setRollerTorque(80, 0.75);
        // setRollerPercent(0.9);
        break;
      case JIGGLE:
        setJiggle();
        // setRollerTorque(80, dynamicIntakeSpeed);
        setRollerPercent(0.2);
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
