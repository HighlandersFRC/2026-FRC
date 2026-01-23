package frc.robot.subsystems.feeder;

import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.subsystems.feeder.Feeder.FeederState;

class FeederIOSim implements FeederIO {
    private double linearizerVelocity = 0.0;
    private double linearizerWantedVelocity = 0.0;
    private double hopperVelocity = 0.0;
    private double hopperWantedVelocity = 0.0;

    @Override
    public void setHopperPercent(double percent) {
        hopperWantedVelocity = percent * Constants.Physical.Feeder.HOPPER_MAX_SPEED_MPS;
    }

    @Override
    public void setLinearizerPercent(double percent) {
        linearizerWantedVelocity = percent * Constants.Physical.Feeder.LINEARIZER_MAX_SPEED_MPS;
    }

    @Override
    public boolean getLinearizerSensorTripped() {
        return OI.driverPOVUp.getAsBoolean();
    }

    @Override
    public void setLinearizerSpeed(double metersPerSecond) {
        linearizerWantedVelocity = metersPerSecond;
    }

    @Override
    public double getLinearizerSpeed() {
        return linearizerVelocity;
    }

    @Override
    public void setHopperSpeed(double metersPerSecond) {
        hopperWantedVelocity = metersPerSecond;
    }

    @Override
    public double getHopperSpeed() {
        return hopperVelocity;
    }

    @Override
    public void updateInputs(FeederState systemState) {
        double dt = Globals.loopPeriodSecs;
        double linearizerAcceleration = Math.signum(linearizerWantedVelocity - linearizerVelocity)
                * Constants.Physical.Feeder.LINEARIZER_ACCELERATION_MPS2;
        double friction = Constants.Physical.Feeder.LINEARIZER_FRICTION_COEFFICIENT * linearizerVelocity;
        linearizerVelocity += (linearizerAcceleration - friction) * dt;
        double hopperAcceleration = Math.signum(hopperWantedVelocity - hopperVelocity)
                * Constants.Physical.Feeder.HOPPER_ACCELERATION_MPS2;
        friction = Constants.Physical.Feeder.HOPPER_FRICTION_COEFFICIENT * hopperVelocity;
        hopperVelocity += (hopperAcceleration - friction) * dt;
    }

    @Override
    public void setHopperTorque(double amps) {

    }

    @Override
    public void setLinearizerTorque(double amps) {

    }
}