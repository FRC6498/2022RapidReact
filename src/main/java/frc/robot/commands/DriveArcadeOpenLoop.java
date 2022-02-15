// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveArcadeOpenLoop extends CommandBase {
  Drivetrain drivetrain;
  DoubleSupplier forward, turn, reverse;
  /** Creates a new DriveArcadeOpenLoop. */
  public DriveArcadeOpenLoop(DoubleSupplier forward, DoubleSupplier turn, DoubleSupplier reverse, Drivetrain drivetrain) {
    this.forward = forward;
    this.turn = turn;
    this.reverse = reverse;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(
      MathUtil.applyDeadband(forward.getAsDouble(), 0.02) + 
      -MathUtil.applyDeadband(reverse.getAsDouble(), 0.02), 
      -turn.getAsDouble()
    );
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
