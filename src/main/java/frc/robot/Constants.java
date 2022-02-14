// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShooterConstants {
        //TODO: look up CAN IDs, perform SysId
        public static final int leftFlywheelCANId = 0;
        public static final int rightFlywheelCANId = 0;
        public static final double flywheelkP = 0;
        public static final double flywheelkD = 0;
        public static final double flywheelkS = 0;
        public static final double flywheelkV = 0;
        public static final double flywheelkA = 0;
        public static final double flywheelVelocityRampRateSeconds = 0;
        public static final double flywheelSetpointToleranceRPM = 10;

    }

    public static final class ConveyorConstants {

        public static final int rearDriverCANId = 0;
        public static final int frontDriverCANId = 0;

    }
}
