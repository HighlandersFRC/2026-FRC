package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

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
    private final TalonFX dyeRotorMotor = new TalonFX(Constants.CANInfo.DYE_ROTOR_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);

    public FeederIOComp() {
        TalonFXConfiguration dyeRotorConfig = new TalonFXConfiguration();
        dyeRotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        dyeRotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        dyeRotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        dyeRotorConfig.CurrentLimits.SupplyCurrentLimit = 90;
        dyeRotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        dyeRotorConfig.Feedback.SensorToMechanismRatio = Constants.Ratios.Feeder.DYE_ROTOR_GEAR_RATIO;
        dyeRotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        dyeRotorMotor.getConfigurator().apply(dyeRotorConfig);
        dyeRotorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setDyeRotorPercent(double percent) {
        dyeRotorMotor.set(percent);
    }

    @Override
    public double getDyeRotorRPM() {
        return dyeRotorMotor.getVelocity()
                .getValueAsDouble()
                * 60.0;
    }

    @Override
    public void updateInputs(FeederState systemState) {
        Logger.recordOutput("Feeder/Dye Rotor Torque", dyeRotorMotor.getStatorCurrent().getValueAsDouble());

    }

    @Override
    public void setDyeRotorTorque(double amps, double maxPercent) {
        dyeRotorMotor.setControl(new TorqueCurrentFOC(amps).withMaxAbsDutyCycle(maxPercent));
    }

}
