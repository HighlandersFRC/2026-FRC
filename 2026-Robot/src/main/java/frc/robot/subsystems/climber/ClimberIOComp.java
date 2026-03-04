package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

class ClimberIOComp implements ClimberIO {
    private TalonFX climberMasterMotor = new TalonFX(Constants.CANInfo.CLIMBER_MASTER_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private TalonFX climberSlaveMotor = new TalonFX(Constants.CANInfo.CLIMBER_SLAVE_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);
    private final TorqueCurrentFOC climberMotorControl = new TorqueCurrentFOC(0.0);

    public ClimberIOComp() {
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.Feedback.SensorToMechanismRatio = 1.0;
        climberConfig.Feedback.RotorToSensorRatio = 1.0;
        climberConfig.CurrentLimits.StatorCurrentLimit = 120;
        climberConfig.CurrentLimits.SupplyCurrentLimit = 120;
        climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberMasterMotor.getConfigurator().apply(climberConfig);
        climberSlaveMotor.getConfigurator().apply(climberConfig);
        climberMasterMotor.setNeutralMode(NeutralModeValue.Brake);
        climberMasterMotor.setPosition(0.0);
        climberSlaveMotor.setNeutralMode(NeutralModeValue.Brake);
        climberSlaveMotor.setPosition(0.0);
    }

    @Override
    public void setPower(double amps, double percent) {
        climberMasterMotor.setControl(climberMotorControl.withOutput(amps).withMaxAbsDutyCycle(percent));
        climberSlaveMotor.setControl(climberMotorControl.withOutput(amps).withMaxAbsDutyCycle(percent));
    }

    @Override
    public double getSlaveCurrent() {
        return climberSlaveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getMasterCurrent() {
        return climberMasterMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void stop() {
        climberMasterMotor.set(0.0);
        climberSlaveMotor.set(0.0);
    }

    @Override
    public double getPosition() {
        return climberMasterMotor.getPosition().getValueAsDouble()
                * Constants.Ratios.Climber.CLIMBER_MOTOR_INCHES_PER_ROTATION;
    }
}
