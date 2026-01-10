package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.SetPoints.ElevatorPosition;
import frc.robot.subsystems.manipulator.Manipulator.ArmItem;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;

  public enum ElevatorState {
    DEFAULT,
    ZERO,
    AUTO_L1,
    AUTO_L2,
    AUTO_L3,
    AUTO_L4,
    AUTO_SCORE_L3,
    AUTO_SCORE_MORE_L3,
    L1,
    L2,
    L3,
    L4,
    FEEDER_INTAKE,
    L2_ALGAE,
    L3_ALGAE,
    GROUND_CORAL_INTAKE,
    GROUND_ALGAE_INTAKE,
    PROCESSOR,
    SCORE_L1,
    SCORE_L2,
    AUTO_SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    NET,
    OVER,
    LOLLIPOP,
    PREHANDOFF,
    HANDOFF
  }

  private double idleTime;
  private boolean firstTimeIdle = true;
  private ElevatorState wantedState = ElevatorState.DEFAULT;
  private ElevatorState systemState = ElevatorState.DEFAULT;
  private double distanceFromL23DriveSetpoint = 0.0;
  private boolean firstTimeDefault = false;

  private ArmItem intakeItem = ArmItem.NONE;

  private boolean initNet = false;
  private boolean initNotNet = false;

  public void updateIntakeItem(ArmItem intakeItem) {
    this.intakeItem = intakeItem;
  }

  public void updateDistanceFromL23DriveSetpoint(double distanceFromL23DriveSetpoint) {
    this.distanceFromL23DriveSetpoint = distanceFromL23DriveSetpoint;
  }

  public double getElevatorL3ScoreSetpoint() {
    if (ElevatorPosition.kAUTOL3.meters - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.2 > 0.0) {
      return (ElevatorPosition.kAUTOL3.meters
          - 3.5 / 39.37 - Math.tan(Math.PI / 6) * distanceFromL23DriveSetpoint * 1.5);
    } else {
      return 0.0;
    }
  }

  public Elevator() {
    if (RobotBase.isReal()) {
      io = new ElevatorIOComp();
    } else {
      io = new ElevatorIOSim();
    }
  }

  public void teleopInit() {
    firstTimeIdle = true;
    firstTimeDefault = false;
    io.teleopInit();
  }

  public void autoInit() {
    io.autoInit();
  }

  public void setCurrentLimit(double stator, double supply) {
    io.setCurrentLimit(stator, supply);
  }

  public void init() {
    io.init();
  }

  public void moveWithPercent(double percent) {
    io.moveWithPercent(percent);
  }

  public void moveWithTorque(double current, double maxPercent) {
    io.moveWithTorque(current, maxPercent);
  }

  public void moveElevatorToPosition(double position) {
    if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      io.setElevatorPosition(position, 0);
    } else {
      if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
        moveWithTorque(18, 0.20);
        // System.out.println("running torque");
      } else {

        io.setElevatorPosition(position, 1);
      }
    }
  }

  public void moveElevatorToPositionSlow(double position) {
    if (position < Constants.Ratios.ELEVATOR_FIRST_STAGE) {
      io.setElevatorPosition(position, 2);
    } else {
      if (position > Constants.inchesToMeters(64.0) && getElevatorPosition() > Constants.inchesToMeters(62.0)) {
        moveWithTorque(18, 0.20);
        // System.out.println("running torque");
      } else {
        io.setElevatorPosition(position, 1);
      }
    }
  }

  public double getElevatorPosition() {
    return io.getElevatorPosition();
  }

  public void setElevatorEncoderPosition(double position) {
    io.setElevatorEncoderPosition(position);
  }

  public void setWantedState(ElevatorState wantedState) {
    this.wantedState = wantedState;
  }

  public LoggedMechanismLigament2d getElevatorLigament() {
    return new LoggedMechanismLigament2d("Elevator", getElevatorPosition(),
        90, 10, new Color8Bit(100, 100, 255));
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ElevatorState.DEFAULT;
      case ZERO:
        return ElevatorState.ZERO;
      case OVER:
        return ElevatorState.OVER;
      case L1:
        return ElevatorState.L1;
      case L2:
        return ElevatorState.L2;
      case L3:
        return ElevatorState.L3;
      case L4:
        return ElevatorState.L4;
      case AUTO_L1:
        return ElevatorState.AUTO_L1;
      case AUTO_L2:
        return ElevatorState.AUTO_L2;
      case AUTO_L3:
        return ElevatorState.AUTO_L3;
      case AUTO_L4:
        return ElevatorState.AUTO_L4;
      case AUTO_SCORE_L3:
        return ElevatorState.AUTO_SCORE_L3;
      case AUTO_SCORE_MORE_L3:
        return ElevatorState.AUTO_SCORE_MORE_L3;
      case FEEDER_INTAKE:
        return ElevatorState.FEEDER_INTAKE;
      case L2_ALGAE:
        return ElevatorState.L2_ALGAE;
      case L3_ALGAE:
        return ElevatorState.L3_ALGAE;
      case GROUND_CORAL_INTAKE:
        return ElevatorState.GROUND_CORAL_INTAKE;
      case GROUND_ALGAE_INTAKE:
        return ElevatorState.GROUND_ALGAE_INTAKE;
      case PROCESSOR:
        return ElevatorState.PROCESSOR;
      case SCORE_L1:
        return ElevatorState.SCORE_L1;
      case SCORE_L2:
        return ElevatorState.SCORE_L2;
      case AUTO_SCORE_L2:
        return ElevatorState.AUTO_SCORE_L2;
      case SCORE_L3:
        return ElevatorState.SCORE_L3;
      case SCORE_L4:
        return ElevatorState.SCORE_L4;
      case NET:
        return ElevatorState.NET;
      case LOLLIPOP:
        return ElevatorState.LOLLIPOP;
      case HANDOFF:
        return ElevatorState.HANDOFF;
      case PREHANDOFF:
        return ElevatorState.PREHANDOFF;
      default:
        return ElevatorState.DEFAULT;
    }
  }

  public boolean getZeroed() {
    if (Math.abs(io.getCurrent()) > 10.0
        && Math.abs(io.getVelocity()) < 5.0) {
      return true;
    } else {
      return false;
    }
  }

  private double zeroTime = 0.0;

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    systemState = handleStateTransition();
    if (systemState != ElevatorState.DEFAULT || OI.driverMenuButton.getAsBoolean()) {
      firstTimeDefault = false;
      idleTime = Timer.getFPGATimestamp();
      zeroTime = 0.0;
    }

    if (systemState != ElevatorState.NET) {
      initNet = false;
      if (!initNotNet) {
        setCurrentLimit(60, 60);
        initNotNet = true;
      }
    }
    // System.out.println("Elevator Current: " +
    // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Elevator Current",
    // elevatorMotorMaster.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Elevator Idle Time", idleTime);
    // Logger.recordOutput("First Time Idle", firstTimeIdle);
    // Logger.recordOutput("Elevator State", systemState);
    Logger.recordOutput("init net", initNet);
    // Logger.recordOutput("Elevator Velocity",
    // Constants.Ratios.elevatorRotationsToMeters(elevatorMotorMaster.getVelocity().getValueAsDouble()));
    // Logger.recordOutput("Elevator Height", getElevatorPosition() * 39.37);
    switch (systemState) {
      case GROUND_CORAL_INTAKE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kGROUNDCORAL.meters);
        break;
      case ZERO:
        moveWithTorque(-40, 0.7);
        if (getZeroed()) {
          setElevatorEncoderPosition(0.0);
        }
        break;
      case NET:
        firstTimeIdle = true;
        if (!initNet) {
          setCurrentLimit(40, 40);
          initNet = true;
          initNotNet = false;
        }
        moveElevatorToPosition(ElevatorPosition.kNET.meters);
        break;
      case PROCESSOR:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kPROCESSOR.meters);
        break;
      case L2_ALGAE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL2ALGAE.meters);
        break;
      case L3_ALGAE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL3ALGAE.meters);
        break;
      case GROUND_ALGAE_INTAKE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kGROUNDALGAE.meters);
        break;
      case OVER:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kOVER.meters);
        break;
      case L1:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL1.meters);
        break;
      case SCORE_L1:
        firstTimeIdle = true;
        break;
      case L2:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL2.meters);
        break;
      case AUTO_SCORE_L2:
        moveElevatorToPosition(ElevatorPosition.kAUTOL2SCORE.meters);
        break;
      case SCORE_L2:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL2.meters - 0.1);
        break;
      case L3:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL3.meters);
        break;
      case SCORE_L3:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL3.meters - 0.2);
        break;
      case L4:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL4.meters);
        break;
      case SCORE_L4:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL4.meters - 10 / 39.37);
        break;
      case FEEDER_INTAKE:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kFEEDER.meters);
        break;
      case AUTO_L1:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kL1.meters);
        break;
      case AUTO_L2:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kAUTOL2.meters);
        break;
      case AUTO_L3:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kAUTOL3.meters);
        break;
      case AUTO_L4:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kAUTOL4.meters);
        break;
      case AUTO_SCORE_MORE_L3:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kAUTOL3SCORE.meters);
        break;
      case AUTO_SCORE_L3:
        firstTimeIdle = true;
        moveElevatorToPosition(getElevatorL3ScoreSetpoint());
        break;
      case LOLLIPOP:
        firstTimeIdle = true;
        moveElevatorToPosition(ElevatorPosition.kLOLLIPOP.meters);
        break;
      case HANDOFF:
        firstTimeDefault = true;
        moveElevatorToPositionSlow(ElevatorPosition.kHANDOFF.meters);
        break;
      case PREHANDOFF:
        firstTimeDefault = true;
        moveElevatorToPosition(ElevatorPosition.kPREHANDOFF.meters);
        break;
      default:
        if (DriverStation.isTeleopEnabled()) {
          // System.out.println("Stupid ahh ts pmo 1");
          if (firstTimeIdle) {
            // System.out.println("Stupid ahh ts pmo 2");
            idleTime = Timer.getFPGATimestamp();
            firstTimeIdle = false;
          }
          if (!firstTimeDefault || OI.driverMenuButton.getAsBoolean()) {
            // System.out.println("Stupid ahh ts pmo 3");
            if (DriverStation.isTeleopEnabled() && Math
                .abs(
                    Constants.Ratios
                        .elevatorRotationsToMeters(
                            io.getVelocity())) < 0.1
                && Timer.getFPGATimestamp() - idleTime > 0.3
                && !firstTimeIdle) {
              // System.out.println("Stupid ahh ts pmo 4");
              if (zeroTime == 0.0) {
                // System.out.println("Stupid ahh ts pmo 5");
                zeroTime = Timer.getFPGATimestamp();
              } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
                // System.out.println("Stupid ahh ts pmo 6");
                firstTimeDefault = true;
                moveWithPercent(0.0);
                setElevatorEncoderPosition(0.0);
              }
            } else {
              // System.out.println("Stupid ahh ts pmo 7");
              // System.out.println("Running down to zero");
              if (intakeItem == ArmItem.ALGAE) {
                // System.out.println("Stupid ahh ts pmo 8");

                moveWithTorque(-40, 0.1);
              } else {
                // System.out.println("Stupid ahh ts pmo 9");
                if (getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
                  moveWithTorque(-50, 1.0);
                  // System.out.println("Stupid ahh ts pmo 25");
                } else if (getElevatorPosition() > (Constants.inchesToMeters(1.0))) {
                  moveWithTorque(-30, 0.4);
                  // System.out.println("Stupid ahh ts pmo 26");
                }
              }
            }
          } else {
            // System.out.println("Stupid ahh ts pmo 10");
            if (getElevatorPosition() > (Constants.inchesToMeters(10.0))) {
              moveWithTorque(-50, 1.0);
              // System.out.println("Stupid ahh ts pmo 11");
            } else if (getElevatorPosition() > (Constants.inchesToMeters(1.0))) {
              moveWithTorque(-30, 0.4);
              // System.out.println("Stupid ahh ts pmo 12");
            } else {
              // System.out.println("Stupid ahh ts pmo 13");
              if (DriverStation.isTeleopEnabled() && Math
                  .abs(
                      Constants.Ratios
                          .elevatorRotationsToMeters(
                              io.getVelocity())) < 0.1
                  && Timer.getFPGATimestamp() - idleTime > 0.3
                  && !firstTimeIdle) {
                // System.out.println("Stupid ahh ts pmo 14");
                if (zeroTime == 0.0) {
                  // System.out.println("Stupid ahh ts pmo 15");
                  zeroTime = Timer.getFPGATimestamp();
                } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
                  // System.out.println("Stupid ahh ts pmo 16");
                  firstTimeDefault = true;
                  moveWithPercent(0.0);
                  setElevatorEncoderPosition(0.0);
                }
              } else {
                // System.out.println("Stupid ahh ts pmo 17");
                // IntakeItem.ALGAE) {

                if (intakeItem == ArmItem.ALGAE) {
                  // System.out.println("Stupid ahh ts pmo 18");
                  moveWithTorque(-40, 0.1);
                } else {
                  // System.out.println("Stupid ahh ts pmo 19");
                  moveWithTorque(-40, 0.4);
                }
              }
            }
          }
        } else {
          // System.out.println("Stupid ahh ts pmo 20");
          if (Math.abs(
              Constants.Ratios.elevatorRotationsToMeters(
                  io.getVelocity())) < 0.1
              && Timer.getFPGATimestamp() - idleTime > 0.3
              && !firstTimeIdle) {
            // System.out.println("Stupid ahh ts pmo 21");
            if (zeroTime == 0.0) {
              // System.out.println("Stupid ahh ts pmo 22");
              zeroTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - zeroTime > 0.5) {
              // System.out.println("Stupid ahh ts pmo 23");
              firstTimeDefault = true;
              moveWithPercent(0.0);
              setElevatorEncoderPosition(0.0);
            }
          } else {
            // System.out.println("Running down to zero"); if (intakeItem ==
            // IntakeItem.ALGAE) {
            // System.out.println("Stupid ahh ts pmo 24");

            moveWithTorque(-40, 0.6);

          }
        }
        break;
    }
  }
}
