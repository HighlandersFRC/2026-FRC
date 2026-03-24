package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder.FeederState;
import frc.robot.tools.logging.TunableNumber;

class FeederIOComp implements FeederIO {
    private final TalonFX dyeRotorMotor = new TalonFX(Constants.CANInfo.DYE_ROTOR_MOTOR_ID,
            Constants.CANInfo.CANBUS_NAME);

    private final VelocityDutyCycle dyeRotorControl = new VelocityDutyCycle(0.0);

    // private TunableNumber feederP = new TunableNumber("Feeder Position kP",
    // Constants.PIDConstants.Feeder.kP0);
    // private TunableNumber feederI = new TunableNumber("Feeder Position kI",
    // Constants.PIDConstants.Feeder.kI0);
    // private TunableNumber feederD = new TunableNumber("Feeder Position kD",
    // Constants.PIDConstants.Feeder.kD0);
    // private TunableNumber feederS = new TunableNumber("Feeder Position kS",
    // Constants.PIDConstants.Feeder.kS0);
    // private TunableNumber feederV = new TunableNumber("Feeder Position kV",
    // Constants.PIDConstants.Feeder.kV0);

    public FeederIOComp() {
        TalonFXConfiguration dyeRotorConfig = new TalonFXConfiguration();
        dyeRotorConfig.Slot0.kP = Constants.PIDConstants.Feeder.kP0;
        dyeRotorConfig.Slot0.kI = Constants.PIDConstants.Feeder.kI0;
        dyeRotorConfig.Slot0.kD = Constants.PIDConstants.Feeder.kD0;
        dyeRotorConfig.Slot0.kS = Constants.PIDConstants.Feeder.kS0;
        dyeRotorConfig.Slot0.kV = Constants.PIDConstants.Feeder.kV0;
        dyeRotorConfig.Slot1.kP = Constants.PIDConstants.Feeder.kP1;
        dyeRotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        dyeRotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        dyeRotorConfig.CurrentLimits.StatorCurrentLimit = 100;
        dyeRotorConfig.CurrentLimits.SupplyCurrentLimit = 100;
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

    boolean useSlot0 = false;

    @Override
    public void setDyeRotorRPM(double rpm) {
        double velocitySetpoint = rpm / 60.0;

        // Logger.recordOutput("Feeder RPM error", rpm - getDyeRotorRPM());
        if (rpm - getDyeRotorRPM() < 10.0) {
            dyeRotorMotor.setControl(dyeRotorControl.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(false));
            useSlot0 = true;
        } else {
            dyeRotorMotor.setControl(dyeRotorControl.withVelocity(velocitySetpoint).withSlot(1).withEnableFOC(false));
            // dyeRotorMotor.set(1.0);
            useSlot0 = false;
        }
        // Logger.recordOutput("Feeder/UsingSlot0", useSlot0);
    }

    @Override
    public double getDyeRotorRPM() {
        return dyeRotorMotor.getVelocity()
                .getValueAsDouble()
                * 60.0;
    }

    @Override
    public void updateInputs(FeederState systemState) {
        // Logger.recordOutput("Feeder/Dye Rotor Torque",
        // dyeRotorMotor.getStatorCurrent().getValueAsDouble());

        // if (feederP.changed() || feederI.changed() || feederD.changed() ||
        // feederS.changed() || feederV
        // .changed()) {
        // TalonFXConfiguration dyeRotorConfig = new TalonFXConfiguration();
        // dyeRotorConfig.Slot0.kP = feederP.get();
        // dyeRotorConfig.Slot0.kI = feederI.get();
        // dyeRotorConfig.Slot0.kD = feederD.get();
        // dyeRotorConfig.Slot0.kS = feederS.get();
        // dyeRotorConfig.Slot0.kV = feederV.get();
        // dyeRotorConfig.Slot1.kP = Constants.PIDConstants.Feeder.kP1;
        // dyeRotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // dyeRotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // dyeRotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        // dyeRotorConfig.CurrentLimits.SupplyCurrentLimit = 90;
        // dyeRotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RotorSensor;
        // dyeRotorConfig.Feedback.SensorToMechanismRatio =
        // Constants.Ratios.Feeder.DYE_ROTOR_GEAR_RATIO;
        // dyeRotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // dyeRotorMotor.getConfigurator().apply(dyeRotorConfig);
        // dyeRotorMotor.setNeutralMode(NeutralModeValue.Brake);
        // }
    }

    @Override
    public void setDyeRotorTorque(double amps, double maxPercent) {
        dyeRotorMotor.setControl(new TorqueCurrentFOC(amps).withMaxAbsDutyCycle(maxPercent));
    }

    @Override
    public double getDyeRotorCurrent() {
        return dyeRotorMotor.getStatorCurrent().getValueAsDouble();
    }
}
