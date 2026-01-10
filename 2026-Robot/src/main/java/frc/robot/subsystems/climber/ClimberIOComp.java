package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberIOComp implements ClimberIO {
    private final TalonFX climberPivot = new TalonFX(Constants.CANInfo.CLIMBER_PIVOT_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

    @Override
    public void init() {
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = 160;
        climberConfig.CurrentLimits.SupplyCurrentLimit = 160;
        climberPivot.getConfigurator().apply(climberConfig);
        climberPivot.setNeutralMode(NeutralModeValue.Brake);
        climberPivot.setPosition(0.0);
    }

    @Override
    public void updateInputs(Climber.ClimbState systemState) {
        // No-op, placeholder for future telemetry if needed
    }

    @Override
    public void setTorque(double current, double maxPercent) {
        climberPivot.setControl(torqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public double getPosition() {
        return climberPivot.getPosition().getValueAsDouble();
    }
}
