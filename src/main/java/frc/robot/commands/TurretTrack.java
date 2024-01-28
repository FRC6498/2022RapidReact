// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.NTHelper;
import frc.robot.subsystems.Turret;

/**
 * Slews until target is aquired and tracks it until it hits a limit switch
 */
public class TurretTrack extends Command {
  Turret turret;
  DoubleSupplier getTargetYaw;
  public TurretTrack(Turret turret, DoubleSupplier getTargetYaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.getTargetYaw = getTargetYaw;
    addRequirements(turret);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d angle = turret.getCurrentPosition().plus(Rotation2d.fromDegrees(-getTargetYaw.getAsDouble()));
    NTHelper.setDouble("turret_setpoint_calc", angle.getDegrees());
    turret.setPositionSetpoint(angle);
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
