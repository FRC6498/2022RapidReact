// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import io.github.oblarg.oblog.Logger;
import static frc.robot.Constants.IntakeConstants.*;

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
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, Constants.ConveyorConstants.frontColorSensorId, Constants.ConveyorConstants.frontConveyorPhotoeyeId);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.rearDriverCANId, Constants.ConveyorConstants.rearColorSensorId, Constants.ConveyorConstants.backConveyorPhotoeyeId);
  Intake frontIntake = new Intake(intakeACANId, frontIntakeForwardChannel, frontIntakeReverseChannel);
  Intake backIntake = new Intake(intakeBCANId, backIntakeForwardChannel, backIntakeReverseChannel);
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret);

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
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    backConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), backConveyor));
    frontIntake.setDefaultCommand(new RunCommand(() -> frontIntake.setMotorSetpoint(0.0), frontIntake));
    backIntake.setDefaultCommand(new RunCommand(() -> backIntake.setMotorSetpoint(0.0), backIntake));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kRightBumper.value).whenActive(new InstantCommand(drivetrain::toggleGear, drivetrain));
    new POVButton(driver, 0).and(superstructure.frontConveyorFull.negate()).whenActive(new ConditionalCommand(new InstantCommand(frontIntake::raiseIntake, frontIntake), new InstantCommand(frontIntake::lowerIntake, frontIntake), frontIntake::isExtended));
    //new POVButton(driver, 90).whenActive(() -> )
    new POVButton(driver, 180).and(superstructure.backConveyorFull.negate()).whenActive(new ConditionalCommand(new InstantCommand(backIntake::raiseIntake, backIntake), new InstantCommand(backIntake::lowerIntake, backIntake), backIntake::isExtended));
  }

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
