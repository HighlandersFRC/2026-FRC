package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeIOComp implements IntakeIO {

    private final TalonFX intakeMotor = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_ID,
            new CANBus(Constants.CANInfo.CANBUS_NAME));

    @Override
    public void init() {

    }

    @Override
    public void updateInputs(IntakeState systemState) {

    }

    @Override
    public void setIntakePercent(double percent) {
        intakeMotor.set(-percent);
    }
}
