// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Vision;
import frc.robot.subsystems.Turret;

/**
 * Slews until target is aquired and tracks it until it hits a limit switch
 */
public class TurretTrack extends CommandBase {
  Turret turret;
  Vision vision;
  boolean forward = false;
  public TurretTrack(Turret turret, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.vision = vision;
    addRequirements(turret);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTargets()) {
      turret.setPositionSetpoint(turret.getCurrentPosition().rotateBy(Rotation2d.fromDegrees(vision.getBestTarget().getYaw())));
    } else {
      if (forward) {
        turret.openLoop(0.2);
      } else {
        turret.openLoop(-0.2);
      }
      if (turret.getFwdLimit()) {
        forward = false;
      }
      if (turret.getRevLimit()) {
        forward = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
