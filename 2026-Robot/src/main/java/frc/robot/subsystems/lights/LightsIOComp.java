package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import frc.robot.Constants;

public class LightsIOComp implements LightsIO {

    private final CANdle candle = new CANdle(Constants.CANInfo.CANDLE_ID_0, "rio");

    @Override
    public void setLEDs(SolidColor color) {
        candle.setControl(color);
    }

    @Override
    public void setLEDs(ColorFlowAnimation animation) {
        candle.setControl(animation);
    }

    @Override
    public void setLEDs(LarsonAnimation animation) {
        candle.setControl(animation);
    }

    @Override
    public void setLEDs(RainbowAnimation animation) {
        candle.setControl(animation);
    }

    @Override
    public void setLEDs(StrobeAnimation animation) {
        candle.setControl(animation);
    }

    @Override
    public void setLEDs(TwinkleAnimation animation) {
        candle.setControl(animation);
    }
}
