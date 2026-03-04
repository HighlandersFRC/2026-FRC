package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeState;

interface IntakeIO {

    public void updateInputs(IntakeState systemState);

    public void setIntakePosition(double rotations);

    public void setRollerPercent(double percent);

    public void setRollerTorque(double amps, double maxPercent);

    public void setPivotTorque(double amps, double maxPercent);

    public double getIntakePosition();

    public double getIntakeVelocity();

    public double getIntakeCurrent();

    public double getIntakeRollerCurrent();

    public double getIntakeAcceleration();

    public void zeroIntakePosition();
}
