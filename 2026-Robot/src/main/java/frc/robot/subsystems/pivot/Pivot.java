package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator.ArmItem;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private double nonAlgaeTime = 0.0;

  private double maxPivotDegrees = 180.0;

  private boolean runManualDownOrUp = false;
  // private Speed fastMode = Speed.FAST;

  public Pivot() {
    if (RobotBase.isReal()) {
      io = new PivotIOComp();
    } else {
      io = new PivotIOSim();
    }
  }

  public void init() {
    io.init();
  }

  private ArmItem intakeItem = ArmItem.NONE;

  public void updateIntakeItem(ArmItem intakeItem) {
    this.intakeItem = intakeItem;
  }

  public void pivotToPosition(double pivotPosition) {
    io.setPosition(pivotPosition, maxPivotDegrees, nonAlgaeTime);
  }

  public void pivotToPositionSlow(double pivotPosition) {
    io.setPositionSlow(pivotPosition, maxPivotDegrees);

  }

  public void pivotToPositionSlower(double pivotPosition) {
    io.setPositionSlower(pivotPosition, maxPivotDegrees);
  }

  public double getPivotPosition() {
    return io.getPosition();
  }

  public void setPivotPercent(double percent) {
    io.setPercent(percent);
  }

  public void setMaxPivotDegrees(double degrees) {
    maxPivotDegrees = degrees;
  }

  public LoggedMechanismLigament2d getLigament() {
    return new LoggedMechanismLigament2d("Pivot", Units.inchesToMeters(29), io.getPosition() * 360);
  }

  public enum PivotFlip {
    FRONT,
    BACK,
  }

  public enum PivotState {
    PREP,
    AUTO_L1,
    AUTO_L2,
    AUTO_L3,
    AUTO_L4,
    L1,
    L23,
    L4,
    PROCESSOR,
    NET,
    FEEDER,
    GROUND_CORAL_FRONT,
    GROUND_CORAL_PREP_BACK,
    GROUND_CORAL_BACK,
    GROUND_ALGAE,
    REEF_ALGAE,
    DEFAULT,
    DEFAULT_CLIMB,
    SCORE_L1,
    SCORE_L23,
    SCORE_L4,
    AUTO_SCORE_L1,
    AUTO_SCORE_L2,
    AUTO_SCORE_L3,
    AUTO_SCORE_L4,
    AUTO_SCORE_L4_SLOW,
    CLIMB,
    UP,
    MANUAL_PLACE,
    MANUAL_RESET,
    IDLE,
    LOLLIPOP,
    HANDOFF
  }

  private PivotState wantedState = PivotState.DEFAULT;
  private PivotState systemState = PivotState.DEFAULT;

  private PivotFlip wantedFlip = PivotFlip.FRONT;
  private PivotFlip systemFlip = PivotFlip.FRONT;

  public void setWantedState(PivotState wantedState) {
    this.wantedState = wantedState;
  }

  public void setWantedFlip(PivotFlip wantedFlip) {
    this.wantedFlip = wantedFlip;
  }

  private PivotFlip handleFlipTransition() {
    switch (wantedFlip) {
      case FRONT:
        return PivotFlip.FRONT;
      case BACK:
        return PivotFlip.BACK;
      default:
        return PivotFlip.FRONT;
    }
  }

  private PivotState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return PivotState.DEFAULT;
      case DEFAULT_CLIMB:
        return PivotState.DEFAULT_CLIMB;
      case UP:
        return PivotState.UP;
      case L1:
        return PivotState.L1;
      case L23:
        return PivotState.L23;
      case L4:
        return PivotState.L4;
      case AUTO_L1:
        return PivotState.AUTO_L1;
      case AUTO_L2:
        return PivotState.AUTO_L2;
      case AUTO_L3:
        return PivotState.AUTO_L3;
      case AUTO_L4:
        return PivotState.AUTO_L4;
      case FEEDER:
        return PivotState.FEEDER;
      case REEF_ALGAE:
        return PivotState.REEF_ALGAE;
      case GROUND_CORAL_FRONT:
        return PivotState.GROUND_CORAL_FRONT;
      case GROUND_CORAL_BACK:
        return PivotState.GROUND_CORAL_BACK;
      case GROUND_CORAL_PREP_BACK:
        return PivotState.GROUND_CORAL_PREP_BACK;
      case GROUND_ALGAE:
        return PivotState.GROUND_ALGAE;
      case PROCESSOR:
        return PivotState.PROCESSOR;
      case NET:
        return PivotState.NET;
      case SCORE_L1:
        return PivotState.SCORE_L1;
      case SCORE_L23:
        return PivotState.SCORE_L23;
      case SCORE_L4:
        return PivotState.SCORE_L4;
      case AUTO_SCORE_L4_SLOW:
        return PivotState.AUTO_SCORE_L4_SLOW;
      case AUTO_SCORE_L1:
        return PivotState.AUTO_SCORE_L1;
      case AUTO_SCORE_L2:
        return PivotState.AUTO_SCORE_L2;
      case AUTO_SCORE_L3:
        return PivotState.AUTO_SCORE_L3;
      case AUTO_SCORE_L4:
        return PivotState.AUTO_SCORE_L4;
      case CLIMB:
        return PivotState.CLIMB;
      case PREP:
        return PivotState.PREP;
      case MANUAL_PLACE:
        return PivotState.MANUAL_PLACE;
      case MANUAL_RESET:
        return PivotState.MANUAL_RESET;
      case IDLE:
        return PivotState.IDLE;
      case LOLLIPOP:
        return PivotState.LOLLIPOP;
      case HANDOFF:
        return PivotState.HANDOFF;
      default:
        return PivotState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    Logger.recordOutput("Pivot Position", getPivotPosition());
    if (systemState != PivotState.L23 && systemState != PivotState.L4 && systemState != PivotState.MANUAL_PLACE
        && systemState != PivotState.MANUAL_RESET) {
      runManualDownOrUp = false;
    }
    if (intakeItem != ArmItem.ALGAE && nonAlgaeTime == 0.0) {
      nonAlgaeTime = Timer.getFPGATimestamp();
    } else if (intakeItem == ArmItem.ALGAE) {
      nonAlgaeTime = 0.0;
    }
    systemState = handleStateTransition();
    systemFlip = handleFlipTransition();
    Logger.recordOutput("Pivot State", systemState);
    switch (systemState) {
      case DEFAULT:
        switch (intakeItem) {
          case ALGAE:
            pivotToPositionSlower(Constants.SetPoints.PivotPosition.kDEFAULT.rotations);

            break;

          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kDEFAULT.rotations);
            break;
        }
        break;
      case DEFAULT_CLIMB:
        pivotToPosition(Constants.SetPoints.PivotPosition.kDEFAULTCLIMB.rotations);
        break;
      case REEF_ALGAE:
        switch (intakeItem) {
          case ALGAE:
            switch (systemFlip) {
              case FRONT:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
              case BACK:
                pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
              default:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
            }

            break;

          default:
            switch (systemFlip) {
              case FRONT:
                pivotToPosition(Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
              case BACK:
                pivotToPosition(-Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
              default:
                pivotToPosition(Constants.SetPoints.PivotPosition.kREEFALGAE.rotations);
                break;
            }
            break;
        }
        break;
      case NET:
        switch (intakeItem) {
          case ALGAE:
            switch (systemFlip) {
              case FRONT:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
              case BACK:
                pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
              default:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
            }

            break;

          default:
            switch (systemFlip) {
              case FRONT:
                pivotToPosition(Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
              case BACK:
                pivotToPosition(-Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
              default:
                pivotToPosition(Constants.SetPoints.PivotPosition.kNET.rotations);
                break;
            }
            break;
        }
        break;
      case PROCESSOR:
        switch (intakeItem) {
          case ALGAE:
            switch (systemFlip) {
              case FRONT:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
              case BACK:
                pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
              default:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
            }

            break;

          default:
            switch (systemFlip) {
              case FRONT:
                pivotToPosition(Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
              case BACK:
                pivotToPosition(-Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
              default:
                pivotToPosition(Constants.SetPoints.PivotPosition.kPROCESSOR.rotations);
                break;
            }
            break;
        }
        break;
      case PREP:
        switch (intakeItem) {
          case ALGAE:
            if (getPivotPosition() > 0) {
              pivotToPositionSlower(Constants.SetPoints.PivotPosition.kPREP.rotations);
            } else {
              pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kPREP.rotations);
            }

            break;

          default:
            if (getPivotPosition() > 0) {
              pivotToPosition(Constants.SetPoints.PivotPosition.kPREP.rotations);
            } else {
              pivotToPosition(-Constants.SetPoints.PivotPosition.kPREP.rotations);
            }
            break;
        }
        break;
      case GROUND_ALGAE:
        switch (intakeItem) {
          case ALGAE:
            switch (systemFlip) {
              case FRONT:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
              case BACK:
                pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
              default:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
            }

            break;

          default:
            switch (systemFlip) {
              case FRONT:
                pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
              case BACK:
                pivotToPosition(-Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
              default:
                pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDALGAE.rotations);
                break;
            }
            break;
        }
        break;
      case UP:
        switch (intakeItem) {
          case ALGAE:
            switch (systemFlip) {
              case FRONT:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
              case BACK:
                pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
              default:
                pivotToPositionSlower(Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
            }

            break;

          default:
            switch (systemFlip) {
              case FRONT:
                pivotToPosition(Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
              case BACK:
                pivotToPosition(-Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
              default:
                pivotToPosition(Constants.SetPoints.PivotPosition.kUP.rotations);
                break;
            }
            break;
        }
        break;
      case GROUND_CORAL_FRONT:
        pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALFRONT.rotations);
        break;
      case LOLLIPOP:
        pivotToPosition(Constants.SetPoints.PivotPosition.kLOLLIPOP.rotations);
        break;
      case GROUND_CORAL_BACK:
        pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALBACK.rotations);
        break;
      case GROUND_CORAL_PREP_BACK:
        pivotToPosition(Constants.SetPoints.PivotPosition.kGROUNDCORALPREPBACK.rotations);
        break;
      case L1:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
        }
        break;
      case CLIMB:
        pivotToPosition(Constants.SetPoints.PivotPosition.kCLIMB.rotations);
        break;
      case SCORE_L1:
        break;
      case L23:
        if (runManualDownOrUp) {
          pivotToPosition(getPivotPosition());
        } else {
          pivotToPosition(Constants.SetPoints.PivotPosition.kL23.rotations);
        }
        break;
      case SCORE_L23:
        pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations);
        break;
      case L4:
        if (runManualDownOrUp) {
          pivotToPosition(getPivotPosition());
        } else {
          pivotToPosition(Constants.SetPoints.PivotPosition.kL4.rotations);
        }
        break;
      case SCORE_L4:
        pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
        break;
      case AUTO_SCORE_L2:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL2SCORE.rotations);
            break;
        }
        break;
      case AUTO_SCORE_L3:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL3SCORE.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL3SCORE.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL3SCORE.rotations);
            break;
        }
        break;

      case AUTO_SCORE_L4:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4SCORE.rotations);
            break;
        }
        break;
      case AUTO_SCORE_L4_SLOW:
        switch (systemFlip) {
          case FRONT:
            pivotToPositionSlower(Constants.SetPoints.PivotPosition.kAUTOL4SCORESLOW.rotations);
            break;
          case BACK:
            pivotToPositionSlower(-Constants.SetPoints.PivotPosition.kAUTOL4SCORESLOW.rotations);
            break;
          default:
            pivotToPositionSlower(Constants.SetPoints.PivotPosition.kAUTOL4SCORESLOW.rotations);
            break;
        }
        break;
      case FEEDER:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kFEEDER.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kFEEDER.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kFEEDER.rotations);
            break;
        }
        break;
      case AUTO_L1:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kL1.rotations);
            break;
        }
        break;
      case AUTO_L2:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL2.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL2.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL2.rotations);
            break;
        }
        break;
      case AUTO_L3:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL3.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL3.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL3.rotations);
            break;
        }
        break;
      case AUTO_L4:
        switch (systemFlip) {
          case FRONT:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
            break;
          case BACK:
            pivotToPosition(-Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
            break;
          default:
            pivotToPosition(Constants.SetPoints.PivotPosition.kAUTOL4.rotations);
            break;
        }
        break;
      case MANUAL_PLACE:
        runManualDownOrUp = true;
        setPivotPercent(0.2);
        break;
      case MANUAL_RESET:
        runManualDownOrUp = true;
        setPivotPercent(-0.2);
        break;
      case HANDOFF:
        pivotToPosition(Constants.SetPoints.PivotPosition.kHANDOFF.rotations);
        break;
      case IDLE:
        setPivotPercent(0.0);
        break;
      default:
        setPivotPercent(0.0);
        break;
    }
  }
}
