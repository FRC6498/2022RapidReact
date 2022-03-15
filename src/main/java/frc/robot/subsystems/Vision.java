// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;


import java.util.ArrayList;
import java.util.Collections;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.NTHelper;
import frc.robot.lib.SortByDistance;

public class Vision extends SubsystemBase {
  PhotonCamera CAM_limelight;
  PhotonPipelineResult currentResult;
  boolean active = true;
  NetworkTable NT_photonvision, NT_limelight;
  /** Creates a new VisionSystem. */
  public Vision() {
    CAM_limelight = new PhotonCamera(limelightCameraName);
    CAM_limelight.setDriverMode(false);
    CAM_limelight.setPipelineIndex(upperHubPipelineID);
    CAM_limelight.setLED(VisionLEDMode.kOff);

    NT_photonvision = NetworkTableInstance.getDefault().getTable("photonvision");
    // THIS IS THE COPROCESSOR NOT FOUND FIX
    NetworkTableEntry photonVersionEntry = NT_photonvision.getEntry("version");
    photonVersionEntry.setString("v2022.1.4");

    NT_limelight = NT_photonvision.getSubTable("limelight");
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
   * 
   * @return Distance to the current pipeline's best target, for input to a PID controller (for shooting)
   */
  public double getTargetDistance(PhotonTrackedTarget target)
  {
    return PhotonUtils.calculateDistanceToTargetMeters(
      Units.inchesToMeters(29), 
      2.64, // 264cm from floor->ring
      Units.degreesToRadians(40), 
      Units.degreesToRadians(target.getPitch())
    );
  }

  public PhotonTrackedTarget getClosestTarget() {
    ArrayList<PhotonTrackedTarget> targets = new ArrayList<>(currentResult.getTargets());
    Collections.sort(targets, new SortByDistance(comparisonConstants));
    return targets.get(0);
  }

  public void setActive(boolean active) {
    this.active = active;
    if (!active) {
      CAM_limelight.setLED(VisionLEDMode.kOff);
    } else {
      CAM_limelight.setLED(VisionLEDMode.kOn);
    }
    
  }

  public int getTargetCount() {
    return currentResult.getTargets().size();
  }

  public double getTargetYaw(PhotonTrackedTarget target) {
    if (hasTargets()) {
      return -target.getYaw();
    } else {
      return 0;
    }
  }

  public VisionLEDMode getLED() {
    return CAM_limelight.getLEDMode();
  }

  public void updatePhotonResult() {
    if (active) {
      currentResult = CAM_limelight.getLatestResult();

      if (currentResult.hasTargets() && currentResult != null) {
        NTHelper.setDouble("target_distance", getTargetDistance(getBestTarget()));
      } else {
        NTHelper.setDouble("target_distance", -1.0);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per robot loop; before triggered commands are scheduled and before any commands are run
  }
}