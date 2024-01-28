// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends Command {
  /** Creates a new FollowTrajectory. */
  Drivetrain drivetrain;
  Trajectory trajectory;
  TrajectoryConfig config;
  PIDController leftController, rightController;
  double distance;
  RamseteController ramsete;
  Timer pathTimer;

  public FollowTrajectory(Drivetrain drivetrain, double distanceMeters) {
    this.distance = distanceMeters;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    ramsete = new RamseteController(DriveConstants.ramseteB, DriveConstants.ramseteZeta);
    config = new TrajectoryConfig(
      DriveConstants.maxSpeedMetersPerSecond, 
      DriveConstants.maxAccelerationMetersPerSecondSquared
    ).setKinematics(drivetrain.getKinematics())
    .addConstraint(new DifferentialDriveVoltageConstraint(
      drivetrain.getFeedforward(), 
      drivetrain.getKinematics(), 
      10)
    );
    pathTimer = new Timer();
    leftController = new PIDController(DriveConstants.kP, 0, 0);
    rightController = new PIDController(DriveConstants.kP, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(distance, 0, new Rotation2d(0))
      ),
      config
    );

    pathTimer.reset();
    pathTimer.start();
    drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the total trajectory time hasnt passed yet, we're still doing it, so run the controller
    if (pathTimer.get() < trajectory.getTotalTimeSeconds()) {
      Trajectory.State sampledState = trajectory.sample(pathTimer.get());
      ChassisSpeeds ramseteSpeeds = ramsete.calculate(drivetrain.getPose(), sampledState);
      // convert chassis speeds to dt speeds
      DifferentialDriveWheelSpeeds ramseteWheelSpeeds = drivetrain.getKinematics().toWheelSpeeds(ramseteSpeeds);
      DifferentialDriveWheelSpeeds measuredWheelSpeeds = drivetrain.getWheelSpeeds();
      drivetrain.tankDriveVolts(
        leftController.calculate(measuredWheelSpeeds.leftMetersPerSecond, ramseteWheelSpeeds.leftMetersPerSecond), 
        rightController.calculate(measuredWheelSpeeds.rightMetersPerSecond, ramseteWheelSpeeds.rightMetersPerSecond)
      );
    } else {
      drivetrain.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathTimer.get() >= trajectory.getTotalTimeSeconds();
  }
}
