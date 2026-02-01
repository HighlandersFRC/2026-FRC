package frc.robot.subsystems.climber;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberIO;

class ClimberIOComp implements ClimberIO {
    private TalonFX climberMotor = new TalonFX(Constants.CANInfo.CLIMBER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
    private final TorqueCurrentFOC climberMotorControl = new TorqueCurrentFOC(0.0);

    public ClimberIOComp() {
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfig.CurrentLimits.StatorCurrentLimit = 80;

        climberMotor.getConfigurator().apply(climberConfig);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setPower(double amps, double percent) {
        climberMotor.setControl(climberMotorControl.withOutput(amps).withMaxAbsDutyCycle(percent));
    }

    @Override
    public void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }
}
