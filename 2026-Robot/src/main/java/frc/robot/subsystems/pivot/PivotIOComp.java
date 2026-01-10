package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot.PivotState;

public class PivotIOComp implements PivotIO {
    private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.PIVOT_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final CANcoder pivotCANcoder = new CANcoder(Constants.CANInfo.PIVOT_CANCODER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    private final double pivotJerk = 0.0;
    private final double pivotAcceleration = 6.0 * Constants.Ratios.PIVOT_GEAR_RATIO;
    private final double pivotCruiseVelocity = 6.0 * Constants.Ratios.PIVOT_GEAR_RATIO;

    private final DynamicMotionMagicVoltage pivotMotionProfileRequest = new DynamicMotionMagicVoltage(0,
            pivotCruiseVelocity,
            pivotAcceleration,
            pivotJerk);

    private final double pivotProfileScalarFactor = 1;

    private final double pivotJerkSlow = 0.0;
    private final double pivotAccelerationSlow = 3.0;
    private final double pivotCruiseVelocitySlow = 3.0;

    private final double pivotJerkSlower = 0.0;
    private final double pivotAccelerationSlower = 1.0;
    private final double pivotCruiseVelocitySlower = 1.0;

    @Override
    public void init() {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 100.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 5.0;
        pivotConfig.Slot1.kP = 30.0;
        pivotConfig.Slot1.kI = 0.0;
        pivotConfig.Slot1.kD = 5.0;
        pivotConfig.Slot2.kP = 50.0;
        pivotConfig.Slot2.kI = 0.0;
        pivotConfig.Slot2.kD = 15.0;
        pivotConfig.MotionMagic.MotionMagicJerk = this.pivotJerk;
        pivotConfig.MotionMagic.MotionMagicAcceleration = this.pivotAcceleration;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = this.pivotCruiseVelocity;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
        pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
        pivotConfig.Feedback.RotorToSensorRatio = Constants.Ratios.PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(PivotState systemState) {
    }

    @Override
    public void setPosition(double rotations, double maxPivotDegrees, double nonAlgaeTime) {
        if (Math.abs(rotations) * 360.0 > maxPivotDegrees) {
            rotations = Math.copySign(maxPivotDegrees / 360.0, rotations);
        }
        if (Timer.getFPGATimestamp() - nonAlgaeTime < 1.0) {
            setPositionSlower(rotations, maxPivotDegrees);
        } else {
            pivotMotor.setControl(this.pivotMotionProfileRequest
                    .withPosition(
                            rotations/* Constants.Ratios.PIVOT_GEAR_RATIO */)
                    .withVelocity(this.pivotCruiseVelocity * pivotProfileScalarFactor)
                    .withAcceleration(this.pivotAcceleration * pivotProfileScalarFactor)
                    .withJerk(
                            this.pivotJerk * pivotProfileScalarFactor)
                    .withSlot(0));
        }
    }

    @Override
    public void setPercent(double percent) {
        pivotMotor.set(percent);
    }

    @Override
    public double getPosition() {
        return (pivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setPositionSlow(double pivotPosition, double maxPivotDegrees) {
        if (Math.abs(pivotPosition) * 360.0 > maxPivotDegrees) {
            pivotPosition = Math.copySign(maxPivotDegrees / 360.0, pivotPosition);
        }
        pivotMotor.setControl(this.pivotMotionProfileRequest
                .withPosition(pivotPosition/* Constants.Ratios.PIVOT_GEAR_RATIO */)
                .withVelocity(this.pivotCruiseVelocitySlow * pivotProfileScalarFactor)
                .withAcceleration(this.pivotAccelerationSlow * pivotProfileScalarFactor)
                .withJerk(
                        this.pivotJerkSlow * pivotProfileScalarFactor)
                .withSlot(0));
    }

    @Override
    public void setPositionSlower(double pivotPosition, double maxPivotDegrees) {
        if (Math.abs(pivotPosition) * 360.0 > maxPivotDegrees) {
            pivotPosition = Math.copySign(maxPivotDegrees / 360.0, pivotPosition);
        }

        pivotMotor.setControl(this.pivotMotionProfileRequest
                .withPosition(pivotPosition/* Constants.Ratios.PIVOT_GEAR_RATIO */)
                .withVelocity(this.pivotCruiseVelocitySlower * pivotProfileScalarFactor)
                .withAcceleration(this.pivotAccelerationSlower * pivotProfileScalarFactor)
                .withJerk(
                        this.pivotJerkSlower * pivotProfileScalarFactor)
                .withSlot(0));
    }

}
