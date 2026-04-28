package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants;

public class LightsIOComp implements LightsIO {

    CANdle candle = new CANdle(0, "rio");

    @Override
    public void setSwerveLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setFrontLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setBackLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void clearSwerveAnimation(int i) {
        candle.clearAnimation(i);
    }

    @Override
    public void clearBackAnimation(int i) {
        candle.clearAnimation(i);
    }

    @Override
    public void clearFrontAnimation(int i) {
        candle.clearAnimation(i);
    }

    @Override
    public void animateSwerve(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void animateBack(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void animateFront(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void setSwerveLEDs(int r, int g, int b, int w, int start, int count) {
        candle.setLEDs(r, g, b, w, start, count);
    }
}
