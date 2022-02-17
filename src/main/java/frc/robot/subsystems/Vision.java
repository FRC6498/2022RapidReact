// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.comparisonConstants;
import static frc.robot.Constants.Vision.lifecamCameraName;
import static frc.robot.Constants.Vision.limelightCameraName;
import static frc.robot.Constants.Vision.upperHubPipelineID;

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
import frc.surpriselib.SortByDistance;
import io.github.oblarg.oblog.Loggable;

public class Vision extends SubsystemBase implements Loggable {
  PhotonCamera CAM_limelight, CAM_lifecam;
  PhotonPipelineResult currentResult;
  boolean active = true;
  NetworkTable NT_photonvision, NT_limelight, NT_lifecam;
  /** Creates a new VisionSystem. */
  public VisionSystem() {
    CAM_limelight = new PhotonCamera(limelightCameraName);
    CAM_limelight.setDriverMode(false);
    CAM_limelight.setPipelineIndex(upperHubPipelineID);
    CAM_limelight.setLED(VisionLEDMode.kOn);

    CAM_lifecam = new PhotonCamera(lifecamCameraName);
    CAM_lifecam.setDriverMode(true);

    NT_photonvision = NetworkTableInstance.getDefault().getTable("photonvision");
    // THIS IS THE COPROCESSOR NOT FOUND FIX
    NetworkTableEntry photonVersionEntry = NT_photonvision.getEntry("version");
    photonVersionEntry.setString("v2022.1.4");

    NT_limelight = NT_photonvision.getSubTable("limelight");
    NT_lifecam = NT_photonvision.getSubTable("Microsoft_LifeCam_HD-3000");
  }

  /**
   * 
   * @param target The target you want data for. Pass in directly from getTarget().
   * @return A double[] with the yaw, pitch, and area values of the target in that order
   */
  public double[] getTargetData()
  {
    PhotonTrackedTarget target = currentResult.getBestTarget();
    double[] data = new double[]
    {
      target.getYaw(),
      target.getPitch(),
      target.getArea()

    };
    return data;
  }

  public void setLED(VisionLEDMode ledMode)
  {
    CAM_limelight.setLED(ledMode);
  }

  public boolean hasTargets() {
    return currentResult.hasTargets();
  }
  /**
   * 
   * @return Distance to the current pipeline's best target, for input to a PID controller (for shooting)
   */
  public double getBestTargetDistance()
  {
    return PhotonUtils.calculateDistanceToTargetMeters(
      comparisonConstants[0], 
      comparisonConstants[1], 
      comparisonConstants[2], 
      Units.degreesToRadians(getTargetData()[1])
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

  @Override
  public void periodic() {
    // This method will be called once per robot loop; before triggered commands are scheduled and before any commands are run
    if (active) {
      currentResult = CAM_limelight.getLatestResult();
    }
  }
}