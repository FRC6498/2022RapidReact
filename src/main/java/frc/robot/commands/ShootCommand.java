// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  Superstructure superstructure;
  public ShootCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addCommands(
    new WaitUntilCommand(() -> superstructure.flyWheelAtSetpoint.get() && superstructure.robotLinedUp.get()),
    new InstantCommand(superstructure::runFeeder, superstructure),
    new InstantCommand(superstructure::runFrontConveyor, superstructure),
    new InstantCommand(superstructure::runRearConveyor, superstructure),
    new RunCommand(() -> {}, superstructure)
    );
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopFrontConveyor();
    superstructure.stopRearConveyor();
    superstructure.stopFeeder();
  }
}
