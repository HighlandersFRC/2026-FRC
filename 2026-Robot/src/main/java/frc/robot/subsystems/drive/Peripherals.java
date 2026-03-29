package frc.robot.subsystems.drive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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

  public List<PhotonPipelineResult> getLeftFrontCamResults() {
    return left_front_cam.getAllUnreadResults();
  }

  public List<PhotonPipelineResult> getLeftBackCamResults() {
    return left_back_cam.getAllUnreadResults();
  }

  public List<PhotonPipelineResult> getRightFrontCamResults() {
    return right_front_cam.getAllUnreadResults();
  }

  public List<PhotonPipelineResult> getRightBackCamResults() {
    return right_back_cam.getAllUnreadResults();
  }

  double cameraScreenshotTime = 0.0;

  public void periodic() {

  }
}
