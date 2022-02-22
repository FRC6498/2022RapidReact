// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double degreesToTurretTicks = 35.04;
    
    public static final class IntakeConstants {
        public static final int intakeACANId = 7;
        public static final int intakeBCANId = 8;
        public static final int frontIntakeForwardChannel = 3;
        public static final int frontIntakeReverseChannel = 2;
        public static final int backIntakeForwardChannel = 4;
        public static final int backIntakeReverseChannel = 5;
    }

    public static final class DriveConstants {
        public static final int leftLeaderCANId = 1;
        public static final int leftFollowerCANId = 2;
        public static final int rightLeaderCANId = 3;
        public static final int rightFollowerCANId = 4;
        public static final int driveRampRate = 1;
        public static final int shifterForwardChannelId = 0;
        public static final int shifterReverseChannelId = 1;
        
    }

    public static final class ShooterConstants {
        //TODO: perform SysId
        public static final int leftFlywheelCANId = 13;
        public static final int rightFlywheelCANId = 14;
        public static final int feederACANId = 10;
        public static final int feederBCANId = 11;     
        public static final double flywheelkP = 0;
        public static final double flywheelkD = 0;
        public static final double flywheelkS = 0;
        public static final double flywheelkV = 0;
        public static final double flywheelkA = 0;
        public static final double flywheelVelocityRampRateSeconds = 0;
        public static final double flywheelSetpointToleranceRPM = 10;
    }

    public static final class TurretConstants {
        public static final int yawMotorCANId = 12;
        public static final double turretPositionToleranceDegrees = 1;
        public static final double turretYaw_kP = 2.4343;
        public static final double turretYaw_kD = 21.027;
        public static final double turretTicksPerRotation = 2048.0*(40.0/10.0)*(40.0/20.0)*(314.0/40.0);
        public static final double turretMaxPosition = 135+13.25;
        public static final double turretMinPosition = -135-13.25;
        public static final double turretSoftLimitOffset = 20;
        public static final double turretHomingVelocityStopThreshold = 0.1;
        public static final double turretFeedforward_kA = 0.89326;
        public static final double turretFeedforward_ks =  5.9704;
        public static final double turretFeedforward_kv =  0.21434;
    }

    public static final class ConveyorConstants {

        public static final int rearDriverCANId = 9;
        public static final int frontDriverCANId = 8;
        public static final int seesawForwardPCMId = 0;
        public static final int seesawReversePCMId = 0;
        public static final int rearColorSensorId = 0;
        public static final int frontColorSensorId = 1;
        public static final double ballPresentCurrentThreshold = 0;
        public static final int frontConveyorPhotoeyeId = 0;
        public static final int backConveyorPhotoeyeId = 0;
        public static final double conveyorNominalSpeed = 0.1;
    }

    public static final class VisionConstants {
        // Heights are in METERS
        // Pitches are in DEGREES (convert to radians)
        public static final double upperHubTargetHeight = Units.inchesToMeters(65);
        public static final double upperHubTargetPitch = Units.degreesToRadians(90);

        // limelight  
        public static final String limelightCameraName = "limelight";
        public static final double limelightHeightFromField = Units.inchesToMeters(27);
        public static final int upperHubPipelineID = 0;
        public static final double limelightPitch = Units.degreesToRadians(33);
        public static final double[] comparisonConstants = new double[] {
            limelightHeightFromField,
            upperHubTargetHeight,
            limelightPitch
        };

        // lifecam
        public static final String lifecamCameraName = "lifecam";
        public static final double lifecamHeightFromField = Units.inchesToMeters(26);
        public static final int redBallPipelineID = 1;
        public static final int blueBallPipelineID = 2;
        public static final double lifecamPitch = Units.degreesToRadians(limelightPitch+180);
    }
}
