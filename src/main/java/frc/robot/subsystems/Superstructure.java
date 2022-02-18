// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.PicoColorSensor;

/**
 * Coordinates all subsystems involving cargo
 * Drivers command Superstructure, which passes it on to the subsystems
 * It makes sure two subsystems are ready for handoff before initiating it
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase {
  // Intake
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor frontConveyor, backConveyor;
  private final PicoColorSensor colorSensor;
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  private final Turret turret;
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

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor, Turret turret, Vision vision) {
    this.flywheel = flywheel;
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
    this.turret = turret;
    this.vision = vision;
    colorSensor = new PicoColorSensor();

    flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setFlywheelIdle(), flywheel));
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    backConveyor.setDefaultCommand(new RunCommand(() -> backConveyor.stop(), backConveyor));

    shooterReady = new Trigger(this::getShooterReady);
    seesawReady = new Trigger();
    frontConveyorFull = new Trigger(frontConveyor::isBallPresent);
    frontConveyorBallColorCorrect = new Trigger(() -> {return frontConveyor.getCargoColor() == this.getAllianceColor(); });
    backConveyorFull = new Trigger(backConveyor::isBallPresent);
    backConveyorBallColorCorrect = new Trigger(() -> {return backConveyor.getCargoColor() == this.getAllianceColor(); });

    setupConveyorCommands();
  }

  public void turnTurret() {
    turret.setDefaultCommand(new RunCommand(()-> turret.turretTurn(), turret));
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
    return shooterAutoEnabled;
  }

  public void setShooterActive(boolean active) {
    shooterAutoEnabled = active;
  }

  public boolean getShooterReady() {
    return flywheel.atSetpoint() && turret.atSetpoint();
  }

  // Methods should be high level actions and command subsystems to achieve the goal
  // TODO: Define what subsytems need, which will inform requirements for this

}
