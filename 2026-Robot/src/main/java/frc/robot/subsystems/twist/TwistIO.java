package frc.robot.subsystems.twist;

import frc.robot.subsystems.twist.Twist.TwistState;

public interface TwistIO {
    public void init();

    public void updateInputs(TwistState systemState);

    public void setPosition(double rotations, int slot);

    public void setPercent(double percent);

    public void setTorque(double torque, double maxPercent);

    public double getPosition();

    public void setEncoderPosition(double position);
}
