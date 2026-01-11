package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class Peripherals {
  private PhotonCamera frontReefCam = new PhotonCamera("Front_Reef");
  private PhotonCamera frontSwerveCam = new PhotonCamera("Front_Swerve");
  private PhotonCamera backReefCam = new PhotonCamera("Back_Reef");
  private PhotonCamera backLeftReefCam = new PhotonCamera("Back_Left_Reef");
  private PhotonCamera backRightReefCam = new PhotonCamera("Back_Right_Reef");
  private PhotonCamera frontBargeCam = new PhotonCamera("Front_Barge");
  private PhotonCamera backBargeCam = new PhotonCamera("Back_Barge");
  private PhotonCamera gamePieceCamera = new PhotonCamera("Front_Game_Piece_Cam");

  AprilTagFieldLayout aprilTagFieldLayout;

  Transform3d robotToCam = new Transform3d(
      new Translation3d(Constants.inchesToMeters(1.75), Constants.inchesToMeters(11.625),
          Constants.inchesToMeters(33.5)),
      new Rotation3d(0, Math.toRadians(30.6), 0));
  PhotonPoseEstimator photonPoseEstimator;

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
          Filesystem.getDeployDirectory().getPath() + "/" + "2025-reefscape.json");
    } catch (Exception e) {
      java.util.logging.Logger.getGlobal().warning("error with april tag: " + e.getMessage());
    }
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
  }

  public void setBackCamPipline(int index) {
    backReefCam.setPipelineIndex(index);
  }

  public void setGamePieceCamPipline(int index) {
    gamePieceCamera.setPipelineIndex(index);
  }

  public PhotonPipelineResult getFrontReefCamResult() {
    var result = frontReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      frontReefCamTrack = true;
      return result.get(0);
    } else {
      frontReefCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackReefCamResult() {
    var result = backReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      backReefCamTrack = true;
      return result.get(0);
    } else {
      backReefCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackLeftReefCamResult() {
    var result = backLeftReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackRightReefCamResult() {
    var result = backRightReefCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontSwerveCamResult() {
    var result = frontSwerveCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontBargeCamResult() {
    var result = frontBargeCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      frontBargeCamTrack = true;
      return result.get(0);
    } else {
      frontBargeCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getBackBargeCamResult() {
    var result = backBargeCam.getAllUnreadResults();
    if (!result.isEmpty()) {
      backBargeCamTrack = true;
      return result.get(0);
    } else {
      backBargeCamTrack = false;
      return new PhotonPipelineResult();
    }
  }

  public PhotonPipelineResult getFrontGamePieceCamResult() {
    var result = gamePieceCamera.getAllUnreadResults();
    if (!result.isEmpty()) {
      return result.get(0);
    } else {
      return new PhotonPipelineResult();
    }
  }

  double cameraScreenshotTime = 0.0;

  public void periodic() {

    // Use to take snapshots of camera stream (Output means processed stream, input
    // means raw stream)
    // if (Timer.getFPGATimestamp() - cameraScreenshotTime > 1.0 &&
    // (DriverStation.isEnabled())) {
    // gamePieceCamera.takeOutputSnapshot();
    // cameraScreenshotTime = Timer.getFPGATimestamp();
    // }

    // Logger.recordOutput("Pidgeon Yaw?", pigeon.getYaw().getValueAsDouble());
    // Logger.recordOutput("Pidgeon Pitch?", pigeon.getPitch().getValueAsDouble());
    // Logger.recordOutput("Pidgeon Roll?", pigeon.getRoll().getValueAsDouble());
    // TODO: uncomment if you want to see if the cameras have a track
    // Logger.recordOutput("Front Cam Track", frontReefCamTrack);
    // Logger.recordOutput("Back Cam Track", backReefCamTrack);
    // Logger.recordOutput("Right Cam Track", frontBargeCamTrack);
    // Logger.recordOutput("Left Cam Track", backBargeCamTrack);
  }
}