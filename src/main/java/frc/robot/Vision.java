// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

@Logged
public class Vision {
  private PhotonCamera CAM_limelight;
  private Optional<PhotonPipelineResult> currentResult = Optional.empty();
  private List<PhotonPipelineResult> allResults;
  boolean enabled = true;  
  /** Creates a new VisionSystem. */
  public Vision() {
    CAM_limelight = new PhotonCamera(VisionConstants.limelightCameraName);
    //PhotonCamera.setVersionCheckEnabled(false);
    CAM_limelight.setDriverMode(false);
    CAM_limelight.setPipelineIndex(VisionConstants.upperHubPipelineID);
  }

  public Optional<PhotonTrackedTarget> getBestTarget()
  {
    if (currentResult.isPresent() && currentResult.get().hasTargets()) {
      return Optional.of(currentResult.get().getBestTarget());
    } else return Optional.empty();
  }

  public void setLED(VisionLEDMode ledMode)
  {
    CAM_limelight.setLED(ledMode);
    //System.out.println(CAM_limelight.getLEDMode().toString());
  }

  public boolean hasTargets() {
    if (currentResult.isPresent()) {
      return currentResult.get().hasTargets();
    } else { return false; }
  }

  /**
   * Algorithm from https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6?u=jonahsnider
   * @param target The vision target to compute the distance to (usually the PV Best Target)
   */
  private double getTargetDistance(PhotonTrackedTarget target)
  {
      // difference in height from the target to camera
      //double dh = 2.64 - Units.inchesToMeters(29);
      //double distance = dh / (Math.tan(target.getPitch()) * Math.cos(target.getYaw()));
      //return distance;
      return PhotonUtils.calculateDistanceToTargetMeters(
          Units.inchesToMeters(29), 
          2.64, // 264cm from floor->ring
          Units.degreesToRadians(40), 
          Units.degreesToRadians(target.getPitch())
      );
  }

  public OptionalDouble getTargetDistance() {
    if (hasTargets()) {
      return OptionalDouble.of(getTargetDistance(currentResult.get().getBestTarget()));
    } else return OptionalDouble.empty();
  }

  public OptionalDouble getTargetYaw() {
    if (hasTargets()) {
      return OptionalDouble.of(currentResult.get().getBestTarget().getYaw());
    } else {
      return OptionalDouble.empty();
    }
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    if (!enabled) {
      CAM_limelight.setLED(VisionLEDMode.kOff);
    } else {
      CAM_limelight.setLED(VisionLEDMode.kOn);
    }
    
  }

  public VisionLEDMode getLEDMode() {
    return CAM_limelight.getLEDMode();
  }

  public boolean getAligned() {
    if (hasTargets() && enabled) {
      return getBestTarget().get().getYaw() < 3;
    } else return false;
  }

  public void setDriverMode(boolean on) {
    CAM_limelight.setDriverMode(on);
  }

  public void periodic() {
    if (enabled) {
      allResults = CAM_limelight.getAllUnreadResults();
      if (!allResults.isEmpty()) {
        currentResult = Optional.of(allResults.get(0));
      }

      /*getTargetDistance().ifPresent(
        (dist) -> log("target_distance", dist)
      );*/
    }
  }
}