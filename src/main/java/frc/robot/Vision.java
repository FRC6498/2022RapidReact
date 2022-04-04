// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.NTHelper;

public class Vision {
  PhotonCamera CAM_limelight;
  PhotonPipelineResult currentResult;
  boolean enabled = true;  
  public Trigger hasTarget;
  /** Creates a new VisionSystem. */
  public Vision() {
    CAM_limelight = new PhotonCamera(limelightCameraName);
    PhotonCamera.setVersionCheckEnabled(false);
    CAM_limelight.setDriverMode(false);
    CAM_limelight.setPipelineIndex(upperHubPipelineID);
    hasTarget = new Trigger(this::hasTargets);
  }

  public PhotonTrackedTarget getBestTarget()
  {
    return currentResult.getBestTarget();
  }

  public void setLED(VisionLEDMode ledMode)
  {
    CAM_limelight.setLED(ledMode);
    //System.out.println(CAM_limelight.getLEDMode().toString());
  }

  public boolean hasTargets() {
    if (currentResult != null) {
      return currentResult.hasTargets();
    } else { return false; }
  }

  /**
   * Algorithm from https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6?u=jonahsnider
   * @param target The vision target to compute the distance to (usually the PV Best Target)
   */
  public double getTargetDistance(PhotonTrackedTarget target)
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
    if (currentResult != null && currentResult.hasTargets() && enabled) {
      return getBestTarget().getYaw() < 3;
    } else return false;
  }

  public void setDriverMode(boolean on) {
    CAM_limelight.setDriverMode(on);
  }

  public void periodic() {
    if (enabled) {
      currentResult = CAM_limelight.getLatestResult();

      if (currentResult.hasTargets() && currentResult != null) {
        NTHelper.setDouble("target_distance", getTargetDistance(getBestTarget()));
      } else {
        NTHelper.setDouble("target_distance", -1.0);
      }
    }
  }
}