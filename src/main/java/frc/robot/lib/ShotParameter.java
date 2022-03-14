// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class ShotParameter {

    // Variables
    public final double rpm;
    public final double offset;

    // Constructor
    public ShotParameter(double rpm, double offset) {
        this.rpm = rpm;
        this.offset = offset;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.rpm - other.rpm) < 0.1 &&
        Math.abs(this.offset - other.offset) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(rpm, end.rpm, t), 
            lerp(offset, end.offset, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}