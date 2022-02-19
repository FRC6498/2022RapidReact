// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ShotMap {
    private TreeMap<Double, Double> map = new TreeMap<>();

    public double getRPM(double distanceMeters) {
        double minDistance = map.firstKey();
        double maxDistance = map.lastKey();
        distanceMeters = MathUtil.clamp(distanceMeters, minDistance, maxDistance); 
        // closest data point to the input that is LESS THAN the input
        Double closestDatumNearer = map.floorKey(distanceMeters);
        // closest data point to the input that is GREATER THAN the input
        Double closestDatumFarther = map.ceilingKey(distanceMeters);
        // Distance between the bounding data points
        double boundDifference = closestDatumFarther - closestDatumNearer;
        // distance to the closest data point inwards
        double realDifference = distanceMeters - closestDatumNearer;
        // distance to closest data point inwards expressed as % of bounding area
        double percent;
        if (boundDifference != 0.0) {
            percent = realDifference / boundDifference;
        } else {
            percent = 0;
        }
        return lerp(percent, closestDatumNearer, closestDatumFarther);
    }

    private double lerp(double percent, double near, double far) {
        return (near * (1.0 - percent)) + (far * percent);
    }

    public void put(double distance, double rpm) {
        map.put(distance, rpm);
    }
}
