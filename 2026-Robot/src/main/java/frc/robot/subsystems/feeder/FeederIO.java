package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.Feeder.FeederState;

interface FeederIO {

    void setDyeRotorPercent(double percent);

    double getDyeRotorRPM();

    double getRollerRPM();

    void setDyeRotorTorque(double amps, double maxPercent);

    void setRollerTorque(double amps, double maxPercent);

    void setDyeRotorRPM(double rpm);

    void updateInputs(FeederState systemState);

    double getDyeRotorCurrent();
}
