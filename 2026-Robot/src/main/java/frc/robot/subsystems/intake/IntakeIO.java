package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeState;

interface IntakeIO {
    public void init();

    public void updateInputs(IntakeState systemState);

    public void setIntakePercent(double percent);
}