package frc.robot.subsystems.twist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator.ArmItem;

public class Twist extends SubsystemBase {
  private final TwistIO io;

  public boolean algaeMode = false;

  public Twist() {
    if (RobotBase.isReal()) {
      io = new TwistIOComp();
    } else {
      io = new TwistIOSim();
    }
  }

  public void init() {
    io.init();
  }

  public void teleopInit() {
  }

  private ArmItem _armItem = ArmItem.NONE;

  public void updateIntakeItem(ArmItem armItem) {
    this._armItem = armItem;
  }

  public void twistToPosition(double rotations) {
    if (_armItem == ArmItem.ALGAE) {
      io.setPosition(rotations, 1);
    } else {
      io.setPosition(rotations, 0);
      ;
    }
  }

  public void setTwistPercent(double percent) {
    io.setPercent(percent);
  }

  public void setTwistTorque(double torque, double maxPercent) {
    io.setTorque(torque, maxPercent);
  }

  public double getTwistPosition() {
    return io.getPosition();
  }

  /**
   * Sets the twist encoder position to a specific value.
   * The position is specified in rotations, where one rotation corresponds to a
   * full 360-degree turn.
   * 
   * @param position The desired encoder position in rotations.
   */
  public void setTwistEncoderPosition(double position) {
    io.setEncoderPosition(position);
  }

  public LoggedMechanismLigament2d getLigament() {
    LoggedMechanismLigament2d mech = new LoggedMechanismLigament2d("Twist", Units.inchesToMeters(6),
        getTwistPosition(), 10, new Color8Bit(255, 255, 100));
    return mech;
  }

  public enum TwistState {
    UP,
    SIDE,
    DOWN,
  }

  private TwistState wantedState = TwistState.SIDE;
  private TwistState systemState = TwistState.SIDE;

  public void setWantedState(TwistState wantedState) {
    this.wantedState = wantedState;
  }

  private TwistState handleStateTransition() {
    switch (wantedState) {
      case UP:
        return TwistState.UP;
      case SIDE:
        return TwistState.SIDE;
      case DOWN:
        return TwistState.DOWN;
      default:
        return TwistState.UP;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(systemState);
    Logger.recordOutput("Twist State: ", systemState);
    Logger.recordOutput("Twist Position", getTwistPosition());
    systemState = handleStateTransition();
    switch (systemState) {
      case DOWN:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_DOWN);
        break;
      case SIDE:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_SIDE);
        break;
      case UP:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_UP);
        break;
      default:
        twistToPosition(Constants.SetPoints.TwistSetpoints.TWIST_DEFAULT);
        break;
    }
  }
}