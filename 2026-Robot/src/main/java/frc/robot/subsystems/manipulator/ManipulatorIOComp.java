package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ManipulatorIOComp implements ManipulatorIO {
    private final TalonFX manipulatorMotor = new TalonFX(Constants.CANInfo.MANIPULATOR_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);

    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    @Override
    public void init() {
        TalonFXConfiguration manipulatorConfig = new TalonFXConfiguration();
        manipulatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        manipulatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        manipulatorConfig.CurrentLimits.StatorCurrentLimit = 80;
        manipulatorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        manipulatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        manipulatorMotor.getConfigurator().apply(manipulatorConfig);
        manipulatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setTorque(double current, double maxPercent) {
        manipulatorMotor.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public double getTorqueCurrent() {
        return manipulatorMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return manipulatorMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getAcceleration() {
        return manipulatorMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        manipulatorMotor.set(percent);
    }

    @Override
    public double getPosition() {
        return manipulatorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return manipulatorMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void updateInputs() {
    }

}
