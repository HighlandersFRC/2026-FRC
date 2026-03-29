package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Peripherals {
  private PhotonCamera left_front_cam = new PhotonCamera("left_front_cam");
  private PhotonCamera left_back_cam = new PhotonCamera("left_back_cam");
  private PhotonCamera right_front_cam = new PhotonCamera("right_front_cam");
  private PhotonCamera right_back_cam = new PhotonCamera("right_back_cam");

  double pigeonSetpoint = 0.0;

  public Peripherals() {
  }

  /**
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
  }

  public PhotonPipelineResult getLeftFrontCamResult() {
    var result = left_front_cam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getLeftBackCamResult() {
    var result = left_back_cam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getRightFrontCamResult() {
    var result = right_front_cam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getRightBackCamResult() {
    var result = right_back_cam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  boolean limelightIsConnected = false;
  double lastBeatTime = 0.0;
  int lastBeatInt = 0;
  public void periodic() {
    int beatCounter = (int) LimelightHelpers.getHeartbeat(Constants.Vision.LIMELIGHT_NAME);
    if (beatCounter == lastBeatInt) {
      limelightIsConnected = false;
    } else if ((System.currentTimeMillis() - lastBeatTime > 10000)) {
      lastBeatTime = System.currentTimeMillis();
      limelightIsConnected = true;
      lastBeatInt = beatCounter;
    }

    Logger.recordOutput("Online/Limelight Online", limelightIsConnected);
    Logger.recordOutput("Online/Left Front Cam Online", left_front_cam.isConnected());
    Logger.recordOutput("Online/Left Back Cam Online", left_back_cam.isConnected());
    Logger.recordOutput("Online/Right Front Cam Online", right_front_cam.isConnected());
    Logger.recordOutput("Online/Right Back Cam Online", right_back_cam.isConnected());
  }
}