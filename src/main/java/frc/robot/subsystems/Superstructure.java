// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  // Feeder


  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor) {
    this.flywheel = flywheel;
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
  }


  public Color getAllianceColor() {
    return null;
  }


  public boolean isBallPresent(int conveyorID) {
    return false;
  }


  public Color getBallColor(int conveyorID) {
    return null;
  }


  public boolean getShooterActive() {
    return false;
  }


  public boolean getShooterReady() {
    return false;
  }

  // Methods should be high level actions and command subsystems to achieve the goal
  // TODO: Define what Superstructure needs, which will inform requirements for subordinate subsystems
}
