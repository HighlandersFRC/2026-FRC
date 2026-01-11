package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeState;

public interface IntakeIO {
    public void init();

    public void updateInputs(IntakeState systemState);

    public void setIntakePosition(double rotations);

    public void setRollerPercent(double percent);

    public double getIntakePosition();
}
