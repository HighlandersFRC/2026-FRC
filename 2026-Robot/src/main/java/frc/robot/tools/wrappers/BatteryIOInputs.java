package frc.robot.tools.wrappers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class BatteryIOInputs implements LoggableInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
    
    @Override
    public void toLog(LogTable table) {
        table.put("BatteryVoltage", batteryVoltage);
        table.put("RioCurrent", rioCurrent);
    }
    @Override
    public void fromLog(LogTable table) {
        batteryVoltage = table.get("BatteryVoltage", 12.0);
        rioCurrent = table.get("RioCurrent", 0.0);
    }
}