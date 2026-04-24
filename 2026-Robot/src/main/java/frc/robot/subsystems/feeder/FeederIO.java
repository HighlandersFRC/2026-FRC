package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.Feeder.FeederState;

interface FeederIO {

    void setDyeRotorPercent(double percent);

    double getDyeRotorRPM();

    double getRollerRPM();

    double getRollerStatorCurrent();

    double getRollerSupplyCurrent();

    void setDyeRotorTorque(double amps, double maxPercent);

    void setRollerTorque(double amps, double maxPercent);

    void setRollerPercent(double maxpercent);

    void setDyeRotorRPM(double rpm);

    void updateInputs(FeederState systemState);

    double getDyeRotorCurrent();
}
