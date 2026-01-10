package frc.robot.subsystems.twist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.twist.Twist.TwistState;

public class TwistIOComp implements TwistIO {
    private final TalonFX twistMotor = new TalonFX(Constants.CANInfo.TWIST_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final CANcoder twistCANcoder = new CANcoder(Constants.CANInfo.TWIST_CANCODER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final double twistJerk = 20.0;
    private final double twistAcceleration = 50.0;
    private final double twistCruiseVelocity = 200.0;

    private final MotionMagicExpoVoltage twistTorqueCurrentFOC = new MotionMagicExpoVoltage(
            0.0);
    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    @Override
    public void init() {
        TalonFXConfiguration twistConfig = new TalonFXConfiguration();
        twistConfig.Slot0.kP = Constants.PIDConstants.Twist.kP0;
        twistConfig.Slot0.kI = Constants.PIDConstants.Twist.kI0;
        twistConfig.Slot0.kD = Constants.PIDConstants.Twist.kD0;
        twistConfig.Slot0.kS = Constants.PIDConstants.Twist.kS0;
        twistConfig.Slot1.kP = Constants.PIDConstants.Twist.kP1;
        twistConfig.Slot1.kI = Constants.PIDConstants.Twist.kI1;
        twistConfig.Slot1.kD = Constants.PIDConstants.Twist.kD1;
        twistConfig.Slot1.kS = Constants.PIDConstants.Twist.kS1;
        twistConfig.MotionMagic.MotionMagicJerk = this.twistJerk;
        twistConfig.MotionMagic.MotionMagicAcceleration = this.twistAcceleration;
        twistConfig.MotionMagic.MotionMagicCruiseVelocity = this.twistCruiseVelocity;
        twistConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        twistConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        twistConfig.CurrentLimits.StatorCurrentLimit = 60;
        twistConfig.CurrentLimits.SupplyCurrentLimit = 60;
        twistConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        twistConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        twistConfig.Feedback.FeedbackRemoteSensorID = twistCANcoder.getDeviceID();
        twistConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.TWIST_GEAR_RATIO_ENCODER;
        twistConfig.Feedback.RotorToSensorRatio = Constants.Ratios.TWIST_GEAR_RATIO_ROTOR;
        twistTorqueCurrentFOC.EnableFOC = true;
        twistMotor.getConfigurator().apply(twistConfig);
        twistMotor.setNeutralMode(NeutralModeValue.Brake);
        twistMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(TwistState systemState) {
    }

    @Override
    public void setPosition(double rotations, int slot) {
        Logger.recordOutput("Twist Target Pos", Constants.rotationsToDegrees(rotations));
        twistMotor.setControl(this.twistTorqueCurrentFOC
                .withPosition(rotations)
                .withEnableFOC(true).withSlot(slot));
    }

    @Override
    public void setPercent(double percent) {
        twistMotor.set(percent);
    }

    @Override
    public void setTorque(double torque, double maxPercent) {
        twistMotor.setControl(torqueCurrentFOCRequest.withOutput(torque).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public double getPosition() {
        return 360.0 * twistMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setEncoderPosition(double position) {
        twistMotor.setPosition(position);
    }
}
