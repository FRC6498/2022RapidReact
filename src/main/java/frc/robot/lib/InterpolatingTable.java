// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

public class InterpolatingTable {

    /* Private constructor because this is a utility class */
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            // distance = 1.97, 2750 rpm
            Map.entry(2.13, new ShotParameter(2100, 0.0)),
            Map.entry(2.54, new ShotParameter(2250, 0.0)),
            Map.entry(3.81, new ShotParameter(2475, 0.0)),
            Map.entry(4.5, new ShotParameter(2550, 200)),
            Map.entry(4.7, new ShotParameter(2675, 200))
        )
    );

    // Method to get shot parameters based on vision distances
    public static ShotParameter get(double distance) {
        Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
        Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
        if (ceilEntry == null) return floorEntry.getValue();
        if (floorEntry == null) return ceilEntry.getValue();
        if (ceilEntry.getValue().equals(floorEntry.getValue())) return ceilEntry.getValue();
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (distance - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    }
}