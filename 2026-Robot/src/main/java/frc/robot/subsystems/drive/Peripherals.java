package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

public class Peripherals {
  private PhotonCamera right_front_cam = new PhotonCamera("right_front_cam");
  private PhotonCamera left_front_cam = new PhotonCamera("left_front_cam");

  AprilTagFieldLayout aprilTagFieldLayout;

  double pigeonSetpoint = 0.0;

  boolean frontReefCamTrack = false;
  boolean backReefCamTrack = false;
  boolean frontBargeCamTrack = false;
  boolean backBargeCamTrack = false;

  public Peripherals() {
  }

  /**
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().getPath() + "/" + "2026-rebuilt.json");
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
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

  public PhotonPipelineResult getLeftFrontCamResult() {
    var result = left_front_cam.getAllUnreadResults();
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