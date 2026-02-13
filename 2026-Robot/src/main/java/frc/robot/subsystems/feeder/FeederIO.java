package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.Feeder.FeederState;

interface FeederIO {

    void setDyeRotorPercent(double percent);

    double getDyeRotorRPM();

    void setDyeRotorTorque(double amps, double maxPercent);

    void updateInputs(FeederState systemState);
}
