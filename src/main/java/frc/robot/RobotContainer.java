// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.commands.HaltConveyor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Drivetrain drivetrain = new Drivetrain();
  Flywheel flywheel = new Flywheel();
  Turret turret = new Turret();
  Vision vision = new Vision();
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, Constants.ConveyorConstants.frontColorSensorId);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.rearDriverCANId, Constants.ConveyorConstants.rearColorSensorId);
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, turret, vision);
  XboxController driver = new XboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    drivetrain.setDefaultCommand(
      new DriveArcadeOpenLoop(
        driver::getRightTriggerAxis, 
        driver::getLeftX, 
        driver::getLeftTriggerAxis, 
        drivetrain
      )
    );
    frontConveyor.setDefaultCommand(new HaltConveyor(frontConveyor));
    backConveyor.setDefaultCommand(new HaltConveyor(backConveyor));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto Started!");
  }
}
