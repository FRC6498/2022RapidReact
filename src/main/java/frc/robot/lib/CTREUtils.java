// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import frc.robot.Constants;

/** Small methods to make working with CTRE motors easier. If we could use C# these would be extension methods but alas. */
public class CTREUtils {

    public double turretTicksToDegrees(double ticks) {
        return ticks / (1 / Constants.degreesToTurretTicks);
    }

    public static double degreesToTurretTicks(double degrees) {
        // 1 rot = 360 degrees
        // 1 rot = 2048 ticks
        // 1 degree = 1/360 rotation
        // 1 degree = 2048 ticks / 360
        // 1 degree = 5.689 ticks
        return degrees * Constants.degreesToTurretTicks;
      }
}
