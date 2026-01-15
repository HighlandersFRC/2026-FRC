package frc.robot.subsystems.feeder;

import frc.robot.subsystems.feeder.Feeder.FeederState;

interface FeederIO {

    void setHopperPercent(double percent);

    void setLinearizerPercent(double percent);

    boolean getLinearizerSensorTripped();

    void setLinearizerSpeed(double metersPerSecond);

    double getLinearizerSpeed();

    void setHopperSpeed(double metersPerSecond);

    double getHopperSpeed();

    void updateInputs(FeederState systemState);
}
