// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import static edu.wpi.first.wpilibj2.command.Commands.*;
/** Add your docs here. */
public class Autos {
    public static Command highGoalOutsideTarmacTimeBased(Intake intake, Conveyor conveyor, Drivetrain drivetrain, Superstructure superstructure) {
        return Commands.sequence(
            runOnce(intake::lowerIntake, intake),
            runOnce(conveyor::start, conveyor),
            run(() -> drivetrain.arcadeDrive(1, 0), drivetrain).withTimeout(0.85),
            runOnce(drivetrain::stop),
            waitSeconds(1),
            runOnce(intake::raiseIntake, intake),
            //new WaitCommand(2),
            superstructure.shoot(false).withTimeout(5),
            run(() -> drivetrain.arcadeDrive(1, 0), drivetrain).withTimeout(0.5)
        );
    }
}
