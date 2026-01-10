package frc.robot.subsystems.manipulator;

public interface ManipulatorIO {

    void init();

    void setTorque(double current, double maxPercent);

    double getTorqueCurrent();

    double getVelocity();

    double getAcceleration();

    void setPercent(double percent);

    double getPosition();

    double getStatorCurrent();

    void updateInputs();
}
