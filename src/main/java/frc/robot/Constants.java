// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    
    public static final class IntakeConstants {
        public static final int intakeACANId = 6;
        public static final int intakeBCANId = 5;
        public static final int frontIntakeForwardChannel = 3;
        public static final int frontIntakeReverseChannel = 2;
        public static final int backIntakeForwardChannel = 4;
        public static final int backIntakeReverseChannel = 5;
    }

    public static final class ClimberConstants{
        public static final int climberMotorCANId = 7;
        public static final double climber_kP = 0.1;
        public static final double climber_kD = 0.0;
    }
    
    public static final class DriveConstants {
        public static final int leftLeaderCANId = 1;
        public static final int leftFollowerCANId = 2;
        public static final int rightLeaderCANId = 3;
        public static final int rightFollowerCANId = 4;
        public static final int driveRampRate = 1;
        public static final int shifterChannelId = 1;
        public static final double kS = 0.56531;
        public static final double kV = 5.7454;
        public static final double kA = 0.32964;
        public static final double kP = 2.4283;
        public static final double trackWidthMeters = Units.inchesToMeters(28.75);
        public static final double maxSpeedMetersPerSecond = 1.9467;
        public static final double maxAccelerationMetersPerSecondSquared = 2;
        public static final double ramseteB = 2;
        public static final double ramseteZeta = 0.7;
        // low gear ratio = 26:1
        // circumference = 0.47877872 meters
        // 2048 ticks * 25 = ticks per 1 wheel rev
        // 53248 ticks/rev => 1 tick = 1/53248 rev
        // 1 tick = 0.000018780048 rev
        // 0.00000899148 meters
        public static final double driveDistancePerTickMeters = 0.00000899148;
        
    }

    public static final class ShooterConstants {        
        public static final int flywheelCANId = 14;
        public static final int feederACANId = 10;
        public static final int feederBCANId = 11;     
        public static final double flywheelkP = 0.1782;
        public static final double flywheelkD = 0;
        public static final double flywheelkS = 0.020432;
        public static final double flywheelkV = 0.12027;
        public static final double flywheelkA = 0.018595;
        public static final double flywheelVelocityRampRateSeconds = 0.5;
        public static final double flywheelSetpointToleranceRPM = 10;
        public static final double flywheelDumpRPM = 500;
        public static final double flywheelHighRPM = 4000;
        public static final int hoodRollerCANId = 13;
    }

    public static final class TurretConstants {
        public static final int yawMotorCANId = 12;
        public static final double turretPositionToleranceDegrees = 0.1;
        public static final double kP = 0.03;
        public static final double kI = 0.0000;
        public static final double kD = 0;
        public static final double turretTicksPerRotation = 2048*(40/10)*(40/20)*(314.0/40.0);
        public static final double turretSoftLimitOffset = 20;
        public static final double turretHomingVelocityStopThreshold = 0.1;
        public static final double kS = 0.89326;
        public static final double kV =  0.020207;
        public static final double kA =  0.00072803;
        public static final double turretTicksPerDegree = turretTicksPerRotation / 360;//254.7;
        public static final double center = 203.31;
        public static final double hardForwardAngle = 231.236-center;
        public static final double hardReverseAngle = 0-center;
        public static final double frontDumpAngle = -5;
        public static final double rearDumpAngle = -178;
    }

    public static final class ConveyorConstants {

        public static final int backDriverCANId = 9;
        public static final int frontDriverCANId = 8;
        public static final int seesawForwardPCMId = 6;
        public static final int seesawReversePCMId = 7;
        public static final int rearColorSensorId = 0;
        public static final int frontColorSensorId = 1;
        public static final double ballPresentCurrentThreshold = 0;
        public static final int frontConveyorPhotoeyeId = 1;
        public static final int backConveyorPhotoeyeId = 0;
        public static final double conveyorNominalSpeed = 1.0;
        public static final double ultrasonicScaleFactor = 1024;
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
        public static final double limelightPitch = Units.degreesToRadians(40);
        
        public static final Pose2d fieldToTargetTransform = new Pose2d(Units.feetToMeters(27), Units.feetToMeters(13.5), Rotation2d.fromDegrees(0));

    }

    public static final double kMaxGoalTrackAge = 0.05;
}
