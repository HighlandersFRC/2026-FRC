package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeState;

class IntakeIOComp implements IntakeIO {
        private final TalonFX pivotMotor = new TalonFX(Constants.CANInfo.INTAKE_PIVOT_MOTOR_ID,
                        new CANBus(Constants.CANInfo.CANBUS_NAME));
        private final TalonFX rollerMotor = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID,
                        new CANBus(Constants.CANInfo.CANBUS_NAME));

        private final double intakeJerk = 0.0;
        private final double intakeAcceleration = 6.0 * Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;
        private final double intakeCruiseVelocity = 6.0 * Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;;

        private final DynamicMotionMagicVoltage intakeMotionProfileRequest = new DynamicMotionMagicVoltage(0,
                        intakeCruiseVelocity,
                        intakeAcceleration);

        private final double intakeProfileScalarFactor = 1;

        @Override
        public void init() {
                pivotMotor.setNeutralMode(NeutralModeValue.Brake);
                TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
                TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

                intakeConfig.Slot0.kP = 100.0;
                intakeConfig.Slot0.kI = 0.0;
                intakeConfig.Slot0.kD = 5.0;
                intakeConfig.Slot1.kP = 30.0;
                intakeConfig.Slot1.kI = 0.0;
                intakeConfig.Slot1.kD = 5.0;
                intakeConfig.Slot2.kP = 50.0;
                intakeConfig.Slot2.kI = 0.0;
                intakeConfig.Slot2.kD = 15.0;
                intakeConfig.MotionMagic.MotionMagicJerk = this.intakeJerk;
                intakeConfig.MotionMagic.MotionMagicAcceleration = this.intakeAcceleration;
                intakeConfig.MotionMagic.MotionMagicCruiseVelocity = this.intakeCruiseVelocity;
                intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                intakeConfig.CurrentLimits.StatorCurrentLimit = 40;
                intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
                intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                intakeConfig.Feedback.SensorToMechanismRatio = 1.0;
                intakeConfig.Feedback.RotorToSensorRatio = Constants.Ratios.Intake.INTAKE_PIVOT_GEAR_RATIO;
                intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                pivotMotor.getConfigurator().apply(intakeConfig);
                pivotMotor.setNeutralMode(NeutralModeValue.Brake);

                rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                rollerConfig.CurrentLimits.StatorCurrentLimit = 40;
                rollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
                rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                rollerMotor.getConfigurator().apply(intakeConfig);
                rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        }

        @Override
        public void updateInputs(IntakeState systemState) {
        }

        @Override
        public void setIntakePosition(double rotations) {

                pivotMotor.setControl(this.intakeMotionProfileRequest
                                .withPosition(rotations)
                                .withVelocity(this.intakeCruiseVelocity * intakeProfileScalarFactor)
                                .withAcceleration(this.intakeAcceleration * intakeProfileScalarFactor)
                                .withJerk(
                                                this.intakeJerk * intakeProfileScalarFactor)
                                .withSlot(0));

        }

        @Override
        public void setRollerPercent(double percent) {
                rollerMotor.set(-percent);
        }

        @Override
        public double getIntakePosition() {
                return (pivotMotor.getPosition().getValueAsDouble());
        }

        @Override
        public void setRollerTorque(double amps) {
                rollerMotor.setControl(new TorqueCurrentFOC(-amps));
        }
}
