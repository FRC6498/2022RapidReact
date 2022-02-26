// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.subsystems.Climber;
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
  Climber climber = new Climber();
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, Constants.ConveyorConstants.frontColorSensorId, Constants.ConveyorConstants.frontConveyorPhotoeyeId);
  Intake frontIntake = new Intake(intakeACANId, frontIntakeForwardChannel, frontIntakeReverseChannel);
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, frontIntake, vision, turret, climber);

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  

  Trigger retractClimb = new Trigger();

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
    drivetrain.setInverted(true);
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    frontIntake.setDefaultCommand(new RunCommand(() -> frontIntake.setMotorSetpoint(0.0), frontIntake));
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
    new POVButton(driver, 0).whileActiveOnce(new StartEndCommand(() -> frontIntake.lowerIntake(), () -> frontIntake.raiseIntake(), frontIntake));
    new POVButton(driver, 270).whenActive(new InstantCommand(frontIntake::reverse, frontIntake));
    new JoystickButton(driver, Button.kA.value).whenActive(new InstantCommand(climber::toggleClimber, climber));
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
