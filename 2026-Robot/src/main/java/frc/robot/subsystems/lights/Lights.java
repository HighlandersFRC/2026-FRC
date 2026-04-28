// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private final LightsIO io;

  private boolean partyMode = false;
  private double strobeSpeed = 0.4;
  private double flashSpeed = 0.05;
  private int ledNumber = 2000;
  private int ledsPerSwerve = 3000;

  StrobeAnimation redFlash = new StrobeAnimation(255, 0, 0, 0, 0.1, ledNumber, 0);
  StrobeAnimation blueFlash = new StrobeAnimation(0, 0, 255, 0, 0.1, ledNumber, 0);

  RainbowAnimation party = new RainbowAnimation(1.0, 1.0, ledNumber, false, 0);

  StrobeAnimation greenStrobe = new StrobeAnimation(0, 255, 0, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation purpleStrobe = new StrobeAnimation(100, 0, 100, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation yellowStrobe = new StrobeAnimation(100, 100, 0, 0, strobeSpeed, ledNumber, 0);

  StrobeAnimation greenFlash = new StrobeAnimation(0, 255, 0, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation purpleFlash = new StrobeAnimation(100, 0, 100, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation yellowFlash = new StrobeAnimation(100, 100, 0, 0, flashSpeed, ledNumber, 0);

  StrobeAnimation algaeFlashing = new StrobeAnimation(0, 75, 25, 0, flashSpeed, ledNumber, 0);
  StrobeAnimation coralFlashing = new StrobeAnimation(50, 50, 50, 50, flashSpeed, ledNumber, 0);
  StrobeAnimation algaeStrobing = new StrobeAnimation(0, 75, 25, 0, strobeSpeed, ledNumber, 0);
  StrobeAnimation coralStrobing = new StrobeAnimation(50, 50, 50, 50, strobeSpeed, ledNumber, 0);

  LarsonAnimation coralKnightRiderAnimation = new LarsonAnimation(255, 255, 255, 0, 0.8, ledNumber,
      BounceMode.Back,
      100, 0);
  LarsonAnimation algaeKnightRiderAnimation = new LarsonAnimation(0, 255, 100, 0, 0.8, ledNumber,
      BounceMode.Back,
      100, 0);

  LarsonAnimation redCylonAnimation = new LarsonAnimation(255, 0, 0, 0, 0.8, ledNumber, BounceMode.Back,
      100, 0);
  LarsonAnimation blueCylonAnimation = new LarsonAnimation(0, 0, 255, 0, 0.8, ledNumber, BounceMode.Back,
      100, 0);

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
    io.setSwerveLEDs(r, g, b);
    io.setBackLEDs(r, g, b);
    io.setFrontLEDs(r, g, b);
  }

  private AllianceState lastAllianceState = AllianceState.RED;

  @Override
  public void periodic() {

    if (OI.isRedSide()) {
      allianceState = AllianceState.RED;
    } else {
      allianceState = AllianceState.BLUE;
    }

    if (allianceState != lastAllianceState) {
      lastAllianceState = allianceState;
      clearAnimations();
    }

    LightsState newState = handleStateTransition();
    if (!DriverStation.isEnabled()) {
      newState = LightsState.DISABLED;
    }
    if (newState != systemState) {
      io.clearSwerveAnimation(0);
      io.clearBackAnimation(0);
      io.clearFrontAnimation(0);
      systemState = newState;
    }

    Logger.recordOutput("Lights/Lights State", systemState);
    Logger.recordOutput("States/Lights State", systemState);
    if (partyMode) {
      weLikeToParty();
    } else {
      if (systemState != LightsState.DISABLED) {
        switch (allianceState) {
          case RED:
            io.setFrontLEDs(100, 0, 0);
            break;
          case BLUE:
            io.setFrontLEDs(0, 0, 100);
            break;
          default:
            io.setFrontLEDs(125, 80, 0);
            break;
        }
      } else {
        switch (allianceState) {
          case RED:
            io.setSwerveLEDs(50, 0, 0);
            break;
          case BLUE:
            io.setSwerveLEDs(0, 0, 50);
            break;
          default:
            break;
        }
      }
      switch (systemState) {
        case DEFAULT:
          switch (allianceState) {
            case RED:
              setRedBouncing();
              break;
            default:
              setBlueBouncing();
              break;
          }
          break;
        case INTAKING:
          setFlashYellow();
          break;
        case SHOOT_PREP:
          setStrobePurple();
          break;
        case SHOOTING:
          setStrobeGreen();
          break;
        default:
          if (OI.autoChooser.isConnected()) {
            switch (allianceState) {
              case RED:
                setRedBouncing();
                break;
              default:
                setBlueBouncing();
                break;
            }
          } else {
            setFlashYellow();
          }
      }
    }
  }

  public void clearAnimations() {
    io.clearSwerveAnimation(0);
    io.clearBackAnimation(0);
    io.clearFrontAnimation(0);
  }

  public void setAllRed() {
    io.setSwerveLEDs(0, 0, 255);
    io.setBackLEDs(0, 0, 255);
    io.setFrontLEDs(0, 0, 255);
  }

  public void setAllBlue() {
    io.setSwerveLEDs(255, 0, 0);
    io.setBackLEDs(255, 0, 0);
    io.setFrontLEDs(255, 0, 0);
  }

  public void setStrobeGreen() {
    io.animateSwerve(greenStrobe);
    io.animateBack(greenStrobe);
  }

  public void setStrobePurple() {
    io.animateSwerve(purpleStrobe);
    io.animateBack(purpleStrobe);
  }

  public void setStrobeYellow() {
    io.animateSwerve(yellowStrobe);
    io.animateBack(yellowStrobe);
  }

  public void setFlashGreen() {
    io.animateSwerve(greenFlash);
    io.animateBack(greenFlash);
    io.animateFront(greenFlash);
  }

  public void setFlashPurple() {
    io.animateSwerve(purpleFlash);
    io.animateBack(purpleFlash);
  }

  public void setFlashYellow() {
    io.animateSwerve(yellowFlash);
    io.animateBack(yellowFlash);
  }

  public void setRedBright() {
    clearAnimations();
    io.setBackLEDs(255, 0, 0);
    io.setSwerveLEDs(255, 0, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setRedDim() {
    clearAnimations();
    io.setBackLEDs(100, 0, 0);
    io.setSwerveLEDs(100, 0, 0, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setBlueBright() {
    clearAnimations();
    io.setBackLEDs(0, 0, 255);
    io.setSwerveLEDs(0, 0, 255, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setBlueDim() {
    clearAnimations();
    io.setBackLEDs(0, 0, 100);
    io.setSwerveLEDs(0, 0, 100, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setWhiteBright() {
    clearAnimations();
    io.setBackLEDs(255, 255, 255);
    io.setSwerveLEDs(255, 255, 255, 255, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setRedFlash() {
    io.animateBack(redFlash);
  }

  public void setBlueFlash() {
    io.animateBack(blueFlash);
  }

  public void setCoralSolid() {
    clearAnimations();
    io.setBackLEDs(30, 30, 30);
    io.setSwerveLEDs(30, 30, 30, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setAlgaeSolid() {
    clearAnimations();
    io.setBackLEDs(0, 75, 25);
    io.setSwerveLEDs(0, 75, 25, 0, ledsPerSwerve * 0, ledsPerSwerve * 2);
  }

  public void setCoralBouncing() {
    io.animateBack(coralKnightRiderAnimation);
  }

  public void setAlgaeBouncing() {
    io.animateBack(algaeKnightRiderAnimation);
  }

  public void setRedBouncing() {
    io.animateSwerve(redCylonAnimation);
    io.animateBack(redCylonAnimation);
    io.animateFront(redCylonAnimation);
  }

  public void setBlueBouncing() {
    io.animateBack(blueCylonAnimation);
    io.animateFront(blueCylonAnimation);
  }

  public void weLikeToParty() {
    io.animateBack(party);
    io.animateFront(party);
    io.animateSwerve(party);
  }

  /**
   * Initializes the Lights subsystem.
   * Clears animation at index 0 of the candle object.
   * 
   */
  public void init() {
    clearAnimations();
  }
}