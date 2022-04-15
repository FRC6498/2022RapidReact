// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShooterMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RejectCargo extends SequentialCommandGroup {
  Superstructure superstructure;
  /** Creates a new RejectCargo. */
  public RejectCargo(Superstructure superstructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.REJECT), superstructure),
      new ShootCommand(superstructure, false),
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure)
    );
    this.superstructure = superstructure;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setShooterMode(ShooterMode.MANUAL_FIRE);
    superstructure.stopFrontConveyor();
    superstructure.stopRearConveyor();
  }
}
