package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Timer;

public class ClimberIOSim implements ClimberIO {
    private double positionRot = 0.0;
    private double requestedCurrent = 0.0;
    private double requestedMaxPct = 0.0;
    private double lastTime = 0.0;

    @Override
    public void init() {
        positionRot = 0.0;
        requestedCurrent = 0.0;
        requestedMaxPct = 0.0;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void updateInputs(Climber.ClimbState systemState) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        // Crude integration: more current and allowed duty => faster change
        double simGain = 0.002; // tune as needed
        positionRot += requestedCurrent * Math.abs(requestedMaxPct) * simGain * dt;
    }

    @Override
    public void setTorque(double current, double maxPercent) {
        this.requestedCurrent = current;
        this.requestedMaxPct = maxPercent;
    }

    @Override
    public double getPosition() {
        return positionRot;
    }
}
