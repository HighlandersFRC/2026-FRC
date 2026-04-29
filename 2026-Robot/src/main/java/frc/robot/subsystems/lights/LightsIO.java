package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;

public interface LightsIO {

    void setLEDs(SolidColor color);

    void setLEDs(ColorFlowAnimation animation);

    void setLEDs(LarsonAnimation animation);

    void setLEDs(RainbowAnimation animation);

    void setLEDs(StrobeAnimation animation);

    void setLEDs(TwinkleAnimation animation);

    void setLEDs(SingleFadeAnimation animation);
}
