package frc.robot.subsystems.feeder;

import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.OI;
import frc.robot.subsystems.feeder.Feeder.FeederState;

class FeederIOSim implements FeederIO {
    private double dyeRotorVelocity = 0.0;
    private double dyeRotorWantedVelocity = 0.0;

    @Override
    public void setDyeRotorPercent(double percent) {
        dyeRotorWantedVelocity = percent * Constants.Physical.Feeder.DYE_ROTOR_MAX_SPEED_MPS;
    }

    @Override
    public double getDyeRotorRPM() {
        return dyeRotorVelocity;
    }

    @Override
    public void updateInputs(FeederState systemState) {
        double dt = Globals.loopPeriodSecs;
        double dyeRotorAcceleration = Math.signum(dyeRotorWantedVelocity - dyeRotorVelocity)
                * Constants.Physical.Feeder.DYE_ROTOR_ACCELERATION_MPS2;
    }

    @Override
    public void setDyeRotorTorque(double amps, double maxPercent) {

    }
}