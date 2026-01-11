package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants;

public class LightsIOComp implements LightsIO {

    CANdle candleSwerve = new CANdle(Constants.CANInfo.CANDLE_ID_0, "Canivore");
    CANdle candleBack = new CANdle(Constants.CANInfo.CANDLE_ID_1, "rio");
    CANdle candleFront = new CANdle(Constants.CANInfo.CANDLE_ID_2, "rio");

    @Override
    public void setSwerveLEDs(int r, int g, int b) {
        candleSwerve.setLEDs(r, g, b);
    }

    @Override
    public void setFrontLEDs(int r, int g, int b) {
        candleFront.setLEDs(r, g, b);
    }

    @Override
    public void setBackLEDs(int r, int g, int b) {
        candleBack.setLEDs(r, g, b);
    }

    @Override
    public void clearSwerveAnimation(int i) {
        candleSwerve.clearAnimation(i);
    }

    @Override
    public void clearBackAnimation(int i) {
        candleBack.clearAnimation(i);
    }

    @Override
    public void clearFrontAnimation(int i) {
        candleFront.clearAnimation(i);
    }

    @Override
    public void animateSwerve(Animation animation) {
        candleSwerve.animate(animation);
    }

    @Override
    public void animateBack(Animation animation) {
        candleBack.animate(animation);
    }

    @Override
    public void animateFront(Animation animation) {
        candleFront.animate(animation);
    }

    @Override
    public void setSwerveLEDs(int r, int g, int b, int w, int start, int count) {
        candleSwerve.setLEDs(r, g, b, w, start, count);
    }
}
