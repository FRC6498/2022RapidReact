// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

public class RunConveyor extends CommandBase {
  Conveyor conveyor;
  //Intake intake;
  Superstructure manager;
  boolean conveyorFull;
  Color ballColor;
  int conveyorID;
  Color allianceColor;

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
  public void initialize() {
    allianceColor = manager.getAllianceColor();
  }

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
    // is the conveyor empty
    conveyorFull = manager.isBallPresent(conveyorID);
    if (conveyorFull) {
      // what ball is in the conveyor
      ballColor = manager.getBallColor(conveyorID);
      
      if (manager.getShooterActive()) {
        // shooter is active, can we shoot this ball
        if (ballColor == allianceColor) {
          // its the correct color
          if (manager.getShooterReady()) {
            conveyor.start();
          }
        }
      }
    } else {
      // we are empty
      // is the intake empty
      /*if (!intake.isBallPresent()) {
        // intake has a ball
        // run conveyor
        conveyor.start();
        // run intake
        intake.startHandoff();

      } else {
        // intake is free
        // stop intake
        intake.stop();
        // conveyor now full or there is no ball, let that bit of execute() take over
        conveyor.stop();
      }*/
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
