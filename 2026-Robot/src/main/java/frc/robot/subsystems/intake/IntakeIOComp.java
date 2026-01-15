package frc.robot.subsystems.intake;

import java.util.logging.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeState;

class IntakeIOComp implements IntakeIO {
        private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.INTAKE_PIVOT_MOTOR_ID,
                        new CANBus(Constants.CANInfo.CANBUS_NAME));
        private final TalonFX rollerMotor = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID,
                        new CANBus(Constants.CANInfo.CANBUS_NAME));

        private final double pivotJerk = 0.0;
        private final double pivotAcceleration = 6.0 * Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;
        private final double pivotCruiseVelocity = 6.0 * Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;;

        private final DynamicMotionMagicVoltage pivotMotionProfileRequest = new DynamicMotionMagicVoltage(0,
                        pivotCruiseVelocity,
                        pivotAcceleration,
                        pivotJerk);

        private final double pivotProfileScalarFactor = 1;

        private final CANrange canRange = new CANrange(0, Constants.CANInfo.CANBUS_NAME);

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
                pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
                pivotConfig.Feedback.RotorToSensorRatio = Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;

                pivotMotor.getConfigurator().apply(pivotConfig);
                pivotMotor.setNeutralMode(NeutralModeValue.Brake);

                CANrangeConfiguration config = new CANrangeConfiguration();
                config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
                config.ProximityParams.ProximityThreshold = 0.1;
                config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

                canRange.getConfigurator().apply(config);
        }

        @Override
        public void updateInputs(IntakeState systemState) {
                org.littletonrobotics.junction.Logger.recordOutput("Can Range (meters)", getCANRangeDistance());
                org.littletonrobotics.junction.Logger.recordOutput("Can Range Signal Strength",
                                getCANRangeSignalStrength());
                org.littletonrobotics.junction.Logger.recordOutput("Can Range Proximity Detected",
                                getCANRangeProximity());
        }

        @Override
        public void setIntakePosition(double rotations) {

                pivotMotor.setControl(this.pivotMotionProfileRequest
                                .withPosition(
                                                rotations/* Constants.Ratios.PIVOT_GEAR_RATIO */)
                                .withVelocity(this.pivotCruiseVelocity * pivotProfileScalarFactor)
                                .withAcceleration(this.pivotAcceleration * pivotProfileScalarFactor)
                                .withJerk(
                                                this.pivotJerk * pivotProfileScalarFactor)
                                .withSlot(0));

        }

        @Override
        public void setRollerPercent(double percent) {
                rollerMotor.set(percent);
        }

        @Override
        public double getIntakePosition() {
                return (pivotMotor.getPosition().getValueAsDouble());
        }

        public double getCANRangeDistance() {
                return canRange.getDistance().getValueAsDouble();
        }

        public double getCANRangeSignalStrength() {
                return canRange.getSignalStrength().getValueAsDouble();
        }

        public boolean getCANRangeProximity() {
                return canRange.getIsDetected().getValue();
        }
}
