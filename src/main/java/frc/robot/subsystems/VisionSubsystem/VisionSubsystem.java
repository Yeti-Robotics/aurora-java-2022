package frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.VisionSubsystem.VisionBackend.CameraMode;
import frc.robot.subsystems.VisionSubsystem.VisionBackend.LEDMode;

public class VisionSubsystem {
  private static VisionBackend visionSubsystem;
  public static VisionAPI api;

  public enum VisionAPI {
    LIMELIGHT,
    PHOTONVISION
  }

  public VisionSubsystem(VisionAPI api) {
    VisionSubsystem.api = api;

    if (api == VisionAPI.LIMELIGHT) {
      visionSubsystem = new LimelightSubsystem();
    } else {
      visionSubsystem = new PhotonVisionSubsystem();
    }
  }

  public static boolean hasTargets() {
    return visionSubsystem.hasTargets();
  }

  public static double getX() {
    return visionSubsystem.getX();
  }

  public static double getY() {
    return visionSubsystem.getY();
  }

  public static double getA() {
    return visionSubsystem.getA();
  }

  public static void setLEDMode(LEDMode mode) {
    visionSubsystem.setLEDMode(mode);
  }

  public static void setCameraMode(CameraMode mode) {
    visionSubsystem.setCameraMode(mode);
  }

  public static void setPipeline(int num) {
    visionSubsystem.setPipeline(num);
  }

  public static double getDistance() {
    return visionSubsystem.getDistance();
  }
}
