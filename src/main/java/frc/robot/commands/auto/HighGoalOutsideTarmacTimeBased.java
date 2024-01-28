// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShooterMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighGoalOutsideTarmacTimeBased extends SequentialCommandGroup {
  /** Creates a new HighGoalOutsideTarmac. */
  public HighGoalOutsideTarmacTimeBased(Superstructure superstructure, Drivetrain drivetrain, Intake intake, Conveyor conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(intake::lowerIntake, intake),
      new InstantCommand(conveyor::start),
      new RunCommand(() -> drivetrain.arcadeDrive(1, 0), drivetrain).withTimeout(0.85),
      new InstantCommand(drivetrain::stop),
      new WaitCommand(1),
      new InstantCommand(intake::raiseIntake, intake),
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE)),
      //new WaitCommand(2),
      superstructure.shoot(false).withTimeout(5),
      new RunCommand(() -> drivetrain.arcadeDrive(1, 0), drivetrain).withTimeout(0.5)
    );
  }
}
