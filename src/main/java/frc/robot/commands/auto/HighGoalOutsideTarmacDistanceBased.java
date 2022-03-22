// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShooterMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighGoalOutsideTarmacDistanceBased extends SequentialCommandGroup {
  public HighGoalOutsideTarmacDistanceBased(Superstructure superstructure, Drivetrain drivetrain, Intake intake) {
    addCommands(
      new InstantCommand(intake::lowerIntake, intake),
      new InstantCommand(() -> drivetrain.slewRateArcadeDrive(0.5, 0), drivetrain),
      new WaitUntilCommand(() -> drivetrain.getDistance(drivetrain.getPose()) >= Units.inchesToMeters(42)),
      new InstantCommand(drivetrain::stop),
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.AUTON)),
      new WaitCommand(4),
      new StartEndCommand(
        superstructure::runFeeder, 
        superstructure::stopFeeder, 
        superstructure
      ).withTimeout(10),
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP))
    );
  }
}
