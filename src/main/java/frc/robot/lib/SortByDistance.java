// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Comparator;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class SortByDistance implements Comparator<PhotonTrackedTarget> {
    double cameraHeight;
    double targetHeight;
    double cameraPitch;
    public SortByDistance(double[] constants) {
        cameraHeight = constants[0];
        targetHeight = constants[1];
        cameraPitch = constants[2];
    }
    @Override
    public int compare(PhotonTrackedTarget arg0, PhotonTrackedTarget arg1) {
        double dist1 = PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeight, 
            targetHeight, 
            cameraPitch, 
            arg0.getPitch()
        );

        double dist2 = PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeight, 
            targetHeight, 
            cameraPitch, 
            arg1.getPitch()
        );

        if (dist1 == dist2) {
            return 0;
        } else if (dist1 > dist2) {
            return 1;
        } else if (dist1 < dist2) {
            return -1;
        }
        
        return 0;
    }

}
