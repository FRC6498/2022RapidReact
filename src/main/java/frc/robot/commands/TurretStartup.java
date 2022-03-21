// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Superstructure.ShooterMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretStartup extends SequentialCommandGroup {
  /** Creates a new TurretStartup. */
  public TurretStartup(Superstructure superstructure, Turret turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.HOMING))
    .andThen(new WaitUntilCommand(turret::getHomed))
    .andThen(() -> superstructure.setShooterMode(ShooterMode.DUMP))
    .andThen(new InstantCommand(() -> turret.setPositionSetpoint(Rotation2d.fromDegrees(TurretConstants.frontDumpAngle)), turret))
    .andThen(new RunCommand(() -> {}, turret)));
  }
}
