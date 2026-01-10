package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public interface ElevatorIO {

    public void updateInputs(ElevatorState systemState);

    public void teleopInit();

    public void autoInit();

    public void setCurrentLimit(double stator, double supply);

    public void init();

    public void moveWithPercent(double percent);

    public void moveWithTorque(double current, double maxPercent);

    public void setElevatorPosition(double position, int slot);

    public double getElevatorPosition();

    public void setElevatorEncoderPosition(double position);

    public double getVelocity();

    public double getCurrent();

}
