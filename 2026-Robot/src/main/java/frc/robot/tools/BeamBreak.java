package frc.robot.tools;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    public final int m_DIOPort;
    private final DigitalInput sensor;

    public BeamBreak(int DIOPort) {
        m_DIOPort = DIOPort;
        sensor = new DigitalInput(DIOPort);
    }

    public boolean isTripped() {
        return !sensor.get();
    }
}