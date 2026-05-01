// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private static final int LED_START_INDEX = 0;
  private static final int LED_END_INDEX = 300;
  private static final int ANIMATION_SLOT = 0;

  private final LightsIO io;

  private boolean partyMode = false;

  public void PARTY() {
    partyMode = true;
  }

  public enum LightsState {
    DISABLED,
    DEFAULT,
    INTAKING,
    SHOOT_PREP,
    SHOOTING
  }

  public enum AllianceState {
    RED,
    BLUE,
  }

  private LightsState wantedState = LightsState.DISABLED;
  private LightsState systemState = LightsState.DISABLED;
  private LightsState lastAppliedState = null;
  private AllianceState allianceState = AllianceState.RED;

  /*
   * Lights codes are as follows:
   * solid yellow - robot can't see auto chooser (might mean that robot is
   * disconected)
   * flashing yellow - error: usually means that the robot cannot see all of its
   * CAN devices and limelights
   * solid red - red alliance
   * solid blue - blue alliance
   * flashing purple:
   * autonomous - the robot does not see the note
   * intaking - robot has not intaken note yet
   * shooting - robot cannot see apriltag
   * flashing green:
   * autonomous - the robot sees the note
   * boot up/CAN check - all CAN is good and limelights are connected
   * intake - robot has intaken note
   * shooting - robot can see apriltag
   * solid green:
   * shooting - robot is aligned and ready to shoot
   */

  /**
   * Constructs a new Lights object.
   * 
   */
  public Lights() {
    if (RobotBase.isReal()) {
      io = new LightsIOComp();
    } else {
      io = new LightsIOSim();
    }
  }

  public void setWantedState(LightsState wantedState) {
    this.wantedState = wantedState;
  }

  private LightsState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return LightsState.DEFAULT;
      case INTAKING:
        return LightsState.INTAKING;
      case SHOOT_PREP:
        return LightsState.SHOOT_PREP;
      case SHOOTING:
        return LightsState.SHOOTING;
      default:
        return LightsState.DISABLED;
    }
  }

  /**
   * Sets the RGB values of the lights to the specified values.
   *
   * @param r The red component value (0-255).
   * @param g The green component value (0-255).
   * @param b The blue component value (0-255).
   */
  public void setCandleRGB(int r, int g, int b) {
    io.setLEDs(new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(rgb(r, g, b)));
  }

  private void applyAnimation() {
    if (partyMode) {
      setPartyRainbow();
      return;
    }

    switch (systemState) {
      case DEFAULT:
        setAllianceBouncing();
        break;
      case INTAKING:
        setIntakingFlow();
        break;
      case SHOOT_PREP:
        setShootPrepTwinkle();
        break;
      case SHOOTING:
        setShootingStrobe();
        break;
      case DISABLED:
      default:
        setDisabledAllianceFlow();
        break;
    }
  }

  private void setDisabledAllianceFlow() {
    io.setLEDs(new ColorFlowAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withColor(allianceColor(65))
        .withFrameRate(12.0));
  }

  private void setAllianceBouncing() {
    io.setLEDs(new LarsonAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withColor(allianceColor(175))
        .withSize(10)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(100.0));
  }

  private void setIntakingFlow() {
    io.setLEDs(new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withColor(rgb(255, 255, 0))
        .withFrameRate(100.0));
  }

  private void setShootPrepTwinkle() {
    io.setLEDs(new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withColor(rgb(255, 0, 255))
        .withFrameRate(300.0));
  }

  private void setShootingStrobe() {
    io.setLEDs(new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withColor(rgb(0, 255, 0))
        .withFrameRate(300.0));
  }

  private void setPartyRainbow() {
    io.setLEDs(new RainbowAnimation(LED_START_INDEX, LED_END_INDEX)
        .withSlot(ANIMATION_SLOT)
        .withBrightness(0.8)
        .withFrameRate(85.0));
  }

  private RGBWColor allianceColor(int brightness) {
    switch (allianceState) {
      case RED:
        return rgb(brightness, 0, 0);
      case BLUE:
      default:
        return rgb(0, 0, brightness);
    }
  }

  private RGBWColor rgb(int r, int g, int b) {
    return new RGBWColor(r, g, b);
  }

  @Override
  public void periodic() {
    // LightsState newState = handleStateTransition();
    // if (!DriverStation.isEnabled()) {
    // newState = LightsState.DISABLED;
    // }

    // systemState = newState;

    // if (systemState != lastAppliedState) {
    // System.out.println("Applying lights state: " + systemState);
    // applyAnimation();
    // lastAppliedState = systemState;
    // }

    // Logger.recordOutput("States/Lights State", systemState);
    // Logger.recordOutput("States/Lights Wanted State", wantedState);
    // Logger.recordOutput("States/Lights Alliance", allianceState);
    // Logger.recordOutput("States/Lights Party Mode", partyMode);
  }

  public void init(AllianceState alliance) {
    allianceState = alliance;
  }
}
