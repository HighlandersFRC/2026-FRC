package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;

public interface LightsIO {

    void setSwerveLEDs(int r, int g, int b);

    void setFrontLEDs(int r, int g, int b);

    void setBackLEDs(int r, int g, int b);

    void clearSwerveAnimation(int i);

    void clearBackAnimation(int i);

    void clearFrontAnimation(int i);

    void animateSwerve(Animation animation);

    void animateBack(Animation animation);

    void animateFront(Animation animation);

    void setSwerveLEDs(int r, int g, int b, int w, int start, int count);

}
