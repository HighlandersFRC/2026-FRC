package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder.FeederState;

public class FeederIOComp implements FeederIO {

    private final TalonFX firstFeederMotor = new TalonFX(Constants.CANInfo.FEEDER_1_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));
    private final TalonFX secondFeederMotor = new TalonFX(Constants.CANInfo.FEEDER_2_MOTOR_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    @Override
    public void init() {

    }

    @Override
    public void updateInputs(FeederState systemState) {

    }

    @Override
    public void setFirstFeederPercent(double percent) {
        firstFeederMotor.set(-percent);
    }

    @Override
    public void setSecondFeederPercent(double percent) {
        secondFeederMotor.set(percent);
    }
}
