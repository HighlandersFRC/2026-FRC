package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;

public class LightsIOSim implements LightsIO {

    @Override
    public void setLEDs(SolidColor color) {
        Logger.recordOutput("Lights/SimControl", color.toString());
    }

    @Override
    public void setLEDs(ColorFlowAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

    @Override
    public void setLEDs(LarsonAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

    @Override
    public void setLEDs(RainbowAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

    @Override
    public void setLEDs(StrobeAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

    @Override
    public void setLEDs(TwinkleAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

    @Override
    public void setLEDs(SingleFadeAnimation animation) {
        Logger.recordOutput("Lights/SimControl", animation.toString());
    }

}
