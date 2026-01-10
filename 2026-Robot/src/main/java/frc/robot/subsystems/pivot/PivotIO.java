package frc.robot.subsystems.pivot;

import frc.robot.subsystems.pivot.Pivot.PivotState;

public interface PivotIO {
    public void init();

    public void updateInputs(PivotState systemState);

    public void setPosition(double rotations, double maxPivotDegrees, double nonAlgaeTime);

    public void setPercent(double percent);

    public double getPosition();

    public void setPositionSlow(double pivotPosition, double maxPivotDegrees);

    public void setPositionSlower(double pivotPosition, double maxPivotDegrees);
}
