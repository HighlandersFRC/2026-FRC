package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder.FeederState;

class FeederIOComp implements FeederIO {
    private final TalonFX hopperMotor = new TalonFX(Constants.CANInfo.HOPPER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
    private final TalonFX linearizerMotor = new TalonFX(Constants.CANInfo.LINEARIZER_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final CANrange linearizerSensor = new CANrange(Constants.CANInfo.LINEARIZER_CANRANGE_ID,
            Constants.CANInfo.CANBUS_NAME);

    public FeederIOComp() {
        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.StatorCurrentLimit = 80;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = 80;
        hopperConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hopperConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Feeder.HOPPER_GEAR_RATIO;
        hopperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hopperMotor.getConfigurator().apply(hopperConfig);
        hopperMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration linearizerConfig = new TalonFXConfiguration();
        linearizerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        linearizerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        linearizerConfig.CurrentLimits.StatorCurrentLimit = 80;
        linearizerConfig.CurrentLimits.SupplyCurrentLimit = 80;
        linearizerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        linearizerConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Feeder.LINEARIZER_GEAR_RATIO;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        linearizerMotor.getConfigurator().apply(linearizerConfig);
        linearizerMotor.setNeutralMode(NeutralModeValue.Brake);

        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ProximityParams.ProximityThreshold = Constants.Physical.Feeder.LINEARIZER_SENSOR_TRIGGER_DISTANCE_M;
        config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        linearizerSensor.getConfigurator().apply(config);
    }

    @Override
    public void setHopperPercent(double percent) {
        hopperMotor.set(percent);
    }

    @Override
    public void setLinearizerPercent(double percent) {
        linearizerMotor
                .set(percent);
    }

    @Override
    public boolean getLinearizerSensorTripped() {
        return linearizerSensor.getIsDetected().getValue();
    }

    @Override
    public void setLinearizerSpeed(double metersPerSecond) {
        double percent = metersPerSecond
                / Constants.Physical.Feeder.LINEARIZER_MAX_SPEED_MPS;
        setLinearizerPercent(percent);
    }

    @Override
    public double getLinearizerSpeed() {
        return linearizerMotor.getVelocity()
                .getValueAsDouble()
                * Constants.Physical.Feeder.LINEARIZER_WHEEL_DIAMETER_M * Math.PI;
    }

    @Override
    public void setHopperSpeed(double metersPerSecond) {
        double percent = metersPerSecond
                / Constants.Physical.Feeder.HOPPER_MAX_SPEED_MPS;
        setHopperPercent(percent);
    }

    @Override
    public double getHopperSpeed() {
        return hopperMotor.getVelocity()
                .getValueAsDouble()
                * Constants.Physical.Feeder.HOPPER_WHEEL_DIAMETER_M * Math.PI;
    }

    @Override
    public void updateInputs(FeederState systemState) {
    }

    @Override
    public void setHopperTorque(double amps, double maxPercent) {
        hopperMotor.setControl(new TorqueCurrentFOC(amps).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public void setLinearizerTorque(double amps, double maxPercent) {
        linearizerMotor.setControl(new TorqueCurrentFOC(amps).withMaxAbsDutyCycle(maxPercent));
    }

}
