package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.Feeder.FeederState;

interface FeederIO {
    public void init();

    public void updateInputs(FeederState systemState);

    public void setFirstFeederPercent(double percent);

    public void setSecondFeederPercent(double percent);
}