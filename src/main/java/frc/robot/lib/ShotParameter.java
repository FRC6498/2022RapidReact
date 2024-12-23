// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import static frc.robot.Constants.ShooterConstants.RotationsPerMinute;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class ShotParameter {

    // Variables
    public final AngularVelocity shooterSpeed;
    public final AngularVelocity hoodSpeedOffset;

    // Constructor
    public ShotParameter(AngularVelocity shooterSpeed, AngularVelocity offset) {
        this.shooterSpeed = shooterSpeed;
        this.hoodSpeedOffset = offset;
    }   

    public ShotParameter(AngularVelocity shooterSpeed) {
        this(shooterSpeed, RotationsPerMinute.of(0));
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.shooterSpeed.minus(other.shooterSpeed).in(RotationsPerMinute)) < 0.1 &&
        Math.abs(this.hoodSpeedOffset.minus(other.hoodSpeedOffset).in(RotationsPerMinute)) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(shooterSpeed, end.shooterSpeed, t), 
            lerp(hoodSpeedOffset, end.hoodSpeedOffset, t)
        );
    }

    // Method lerp
    private AngularVelocity lerp(AngularVelocity y2, AngularVelocity y1, double t) {
        return y1.plus((y2.minus(y1)).times(t));
    }
 
}