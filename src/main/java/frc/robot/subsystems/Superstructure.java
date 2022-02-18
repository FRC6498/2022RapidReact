// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.PicoColorSensor;
import frc.robot.lib.PicoColorSensor.RawColor;

import static frc.robot.Constants.ConveyorConstants.*;

import java.util.ArrayList;
import java.util.Map;

import com.revrobotics.ColorSensorV3;

/**
 * Coordinates all subsystems involving cargo
 * Drivers command Superstructure, which passes it on to the subsystems
 * It makes sure two subsystems are ready for handoff before initiating it
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase {
  // Intake
  // Vision
  // Conveyor
  private final Conveyor frontConveyor, backConveyor;
  private final PicoColorSensor colorSensor;
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  // Feeder
  boolean shooterAutoEnabled;
  // Triggers
  // Superstructure
  Trigger shooterReady;
  Trigger seesawReady;
  // Conveyors
  Trigger frontConveyorFull;
  Trigger frontConveyorBallColorCorrect;
  Trigger backConveyorFull;
  Trigger backConveyorBallColorCorrect;
  // Intakes
  Trigger frontIntakeFull;
  Trigger backIntakeFull;

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor) {
    this.flywheel = flywheel;
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
    colorSensor = new PicoColorSensor();

    shooterReady = new Trigger(this::getShooterReady);
    seesawReady = new Trigger();
    frontConveyorFull = new Trigger(frontConveyor::isBallPresent);
    frontConveyorBallColorCorrect = new Trigger(() -> {return frontConveyor.getCargoColor() == this.getAllianceColor(); });
    backConveyorFull = new Trigger(backConveyor::isBallPresent);
    backConveyorBallColorCorrect = new Trigger(() -> {return backConveyor.getCargoColor() == this.getAllianceColor(); });
    frontIntakeFull = new Trigger();
    backIntakeFull = new Trigger();

    setupConveyorCommands();
  }

  private void setupConveyorCommands() {
    // move to seesaw
    shooterReady.and(frontConveyorFull).and(frontConveyorBallColorCorrect).and(seesawReady).whileActiveOnce(
      new StartEndCommand(
        frontConveyor::start, 
        frontConveyor::stop, 
        frontConveyor
      )
    );
    shooterReady.and(backConveyorFull).and(backConveyorBallColorCorrect).and(seesawReady).whileActiveOnce(
      new StartEndCommand(
        backConveyor::start, 
        backConveyor::stop, 
        backConveyor
      )
    );

    // grab from intake
    frontIntakeFull.and(frontConveyorFull.negate()).whileActiveOnce(
      new ParallelCommandGroup(
        new StartEndCommand(frontIntake::startHandoff, frontIntake::stop, frontConveyor, frontIntake),
        new StartEndCommand(frontConveyor::start, frontConveyor::stop, frontConveyor)
      )
    );
    backIntakeFull.and(backConveyorFull.negate()).whileActiveOnce(
      new ParallelCommandGroup(
        new StartEndCommand(backIntake::startHandoff, backIntake::stop, backConveyor, backIntake),
        new StartEndCommand(backConveyor::start, backConveyor::stop, backConveyor)
      )
    );
  }

  public Color getAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    switch (alliance) {
      case Red:
        return Color.kRed;
      case Blue:
        return Color.kBlue;
      case Invalid:
      default:
        return Color.kBrown;
    }
  }

  public void getBallColors() {
    backConveyor.setCargoColor(colorSensor.getColor(backConveyor.colorSensorId));
    frontConveyor.setCargoColor(colorSensor.getColor(frontConveyor.colorSensorId));
  }


  public boolean getShooterActive() {
    return false;
  }

  public boolean feederIsOccupied() {
    return false;
  }

  public boolean getShooterReady() {
    return false;
  }

  // Methods should be high level actions and command subsystems to achieve the goal
  // TODO: Define what subsytems need, which will inform requirements for this

}
