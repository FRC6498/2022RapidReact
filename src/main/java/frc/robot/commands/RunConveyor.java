// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Superstructure;

public class RunConveyor extends CommandBase {
  Conveyor conveyor;
  //Intake intake;
  Superstructure manager;
  boolean conveyorFull;
  Color ballColor;
  int conveyorID;

  /** Creates a new RunConveyor. */
  public RunConveyor(Conveyor conveyor, Superstructure superstructure, int conveyorID) {
    this.conveyor = conveyor;
    //this.intake = intake;
    manager = superstructure;
    addRequirements(conveyor);
    this.conveyorID = conveyorID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Perform Clearance Checks
  // is intake full?
  // Is conveyor full?
  // is seesaw full?

  // Conditions
  // Is Shooter on?
  // are we defending?
  // Is this ball our color?
  // Is Seesaw aligned?



  @Override
  public void execute() {
    // can we accept a ball
    //conveyorFull = manager.getBallPresent(conveyorID);
    if (conveyorFull) {
      
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
