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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.subsystems.Superstructure.ShooterMode;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.Consumer;

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
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, 0);
  Intake frontIntake = new Intake(intakeACANId, frontIntakeForwardChannel, frontIntakeReverseChannel);
  Consumer<ShooterMode> shooterModeUpdater = (ShooterMode mode) -> {
    flywheel.setShooterMode(mode);
    turret.setShooterMode(mode);
  };
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, frontIntake, vision, turret, climber, shooterModeUpdater);

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  Trigger retractClimb = new Trigger();
  Trigger flyWheelAtSetpoint = new Trigger();

  JoystickButton driver_rbumper = new JoystickButton(driver, Button.kRightBumper.value);
  JoystickButton driver_a = new JoystickButton(driver, Button.kA.value);
  JoystickButton op_b = new JoystickButton(driver, Button.kB.value);
  POVButton op_up = new POVButton(operator, 0);
  POVButton op_left = new POVButton(operator, 270);
  JoystickButton driver_lBumper = new JoystickButton(driver, Button.kLeftBumper.value);
  public boolean feederRunning;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
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
    driver_rbumper.whenActive(new InstantCommand(drivetrain::toggleGear, drivetrain));
    op_up.whileActiveOnce(new StartEndCommand(() -> frontIntake.lowerIntake(), () -> frontIntake.raiseIntake(), frontIntake));
    op_up.whenActive(new ConditionalCommand(
      new InstantCommand(frontIntake::raiseIntake, frontIntake), // intake down, so raise it
      new SequentialCommandGroup( // intake up, so lower it
        new InstantCommand(frontIntake::lowerIntake, frontIntake),
        new WaitCommand(5) 
      ),  frontIntake::isExtended));
    op_left.whenActive(new InstantCommand(frontIntake::reverse, frontIntake));
    driver_a.whenActive(new InstantCommand(climber::toggleClimber, climber));
    climber.setDefaultCommand(new RunCommand(() -> climber.setInput(driver.getRightY() / 2), climber));
    op_b.whileActiveOnce(new StartEndCommand(superstructure::runFeeder, superstructure::stopFeeder, superstructure));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      // drive forward and intake
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          new RunCommand(() -> drivetrain.arcadeDrive(0.5, 0), drivetrain),
          new WaitUntilCommand(1)
        ),
        new StartEndCommand(frontIntake::lowerIntake, frontIntake::raiseIntake, frontIntake).until(() -> frontConveyor.isBallPresent())
      ),
      // set dump mode (we will go for the fender)
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP)),
      // drive back to fender
      new ParallelCommandGroup(
        new RunCommand(() -> drivetrain.arcadeDrive(-0.5, 0)).until(drivetrain::getStopped)
      ),
      // send balls into shooter until conveyor is empty
      new StartEndCommand(superstructure::runFeeder, superstructure::stopFeeder, superstructure).until(() -> frontConveyor.isBallPresent() == false)
    );
  }
}
