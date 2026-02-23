package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Peripherals {
  private PhotonCamera left_front_cam = new PhotonCamera("left_front_cam");
  private PhotonCamera left_back_cam = new PhotonCamera("left_back_cam");

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

  double cameraScreenshotTime = 0.0;

  public void periodic() {

  }
}