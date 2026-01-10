package frc.robot.subsystems.intake;

public interface IntakeIO {

    void init();

    void setPivotTorque(double current, double maxPercent);

    void setPivotPosition(double pivotRotations);

    void setRollerCurrent(double amps, double maxPercent);

    void setRollerPercent(double percent);

    double getPivotPosition();

    double getPivotStatorCurrent();

    double getPivotVelocity();

    double getRollerStatorCurrent();

    double getRollerVelocity();

    void setPivotEncoderPosition(double d);

    void setPivotCurrent(double amps, double maxPercent);

    double getRollerTorqueCurrent();
}
