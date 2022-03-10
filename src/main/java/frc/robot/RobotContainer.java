// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Special Thanks To:
// 832 for Superstructure assistance
// 449 for Oblarg
// 3512 for Tyler Veness (and feedforwards/sysid)
// 4390 and 4272 for picking us at Kokomo

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.commands.TurretHoming;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.Consumer;

import org.photonvision.common.hardware.VisionLEDMode;

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
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.backDriverCANId, 1);
  Intake frontIntake = new Intake(intakeACANId, frontIntakeForwardChannel, frontIntakeReverseChannel);
  Intake backIntake = new Intake(intakeBCANId, backIntakeForwardChannel, backIntakeReverseChannel);
  Consumer<ShooterMode> shooterModeUpdater = (ShooterMode mode) -> {
    flywheel.setShooterMode(mode);
    turret.setShooterMode(mode);
  };
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret, climber, shooterModeUpdater);

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  Trigger retractClimb = new Trigger();
  Trigger flyWheelAtSetpoint = new Trigger();
  @Log(tabName = "SmartDashboard")
  SendableChooser<Command> autoSelector = new SendableChooser<>();

  JoystickButton driver_rbumper = new JoystickButton(driver, Button.kRightBumper.value);
  JoystickButton driver_a = new JoystickButton(driver, Button.kA.value);
  JoystickButton driver_b = new JoystickButton(driver, Button.kB.value);
  JoystickButton operator_a = new JoystickButton(operator, Button.kA.value);
  JoystickButton operator_x = new JoystickButton(operator, Button.kX.value);
  POVButton op_up = new POVButton(operator, 0);
  POVButton op_left = new POVButton(operator, 270);
  POVButton op_down = new POVButton(operator, 180);
  POVButton op_right = new POVButton(operator, 90);
  JoystickButton driver_lBumper = new JoystickButton(driver, Button.kLeftBumper.value);
  public boolean feederRunning;

  SequentialCommandGroup lowAuto = new ParallelCommandGroup(
    // flywheel ALWAYS spinning
    new RunCommand(() -> flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelDumpRPM), flywheel),
    //new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP), superstructure),
    new SequentialCommandGroup(
      // send balls into shooter until conveyor is empty
      new ParallelRaceGroup(
        new StartEndCommand(superstructure::runFeeder, superstructure::stopFeeder, superstructure),
        new WaitCommand(5)
      ),
      new InstantCommand(drivetrain::stop, drivetrain)
    )
  ).andThen(() -> drivetrain.stop(), drivetrain);

  SequentialCommandGroup lowAutoDecorated = 
    new InstantCommand(() -> flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelDumpRPM), flywheel)
    .andThen(
      new StartEndCommand(
        superstructure::runFeeder, 
        superstructure::stopFeeder, 
        superstructure
      ).withTimeout(5)
    ).andThen(
      new RunCommand(() -> drivetrain.arcadeDrive(-1, 0), drivetrain).withTimeout(1.5)
    ).andThen(drivetrain::stop);

  SequentialCommandGroup highAutoDecorated = 
    new InstantCommand(() -> flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelHighRPM), flywheel)
    .andThen(
      new WaitUntilCommand(flywheel::atSetpoint)
    ).andThen(
      new StartEndCommand(
        superstructure::runFeeder, 
        superstructure::stopFeeder, 
        superstructure
      )
    ).andThen(
      new RunCommand(() -> drivetrain.arcadeDrive(-1, 0), drivetrain).withTimeout(1.5)
    ).andThen(drivetrain::stop);

    SequentialCommandGroup turretCmd = 
      new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.HOMING))
      .andThen(turret::setForward)
      .andThen(new RunCommand(() -> turret.openLoop(-0.1), turret))
      .until(() -> turret.getRevLimit() )
      //.andThen(turret::setInverted)
      .andThen(
        new InstantCommand(turret::resetSensor, turret)
        //new RunCommand(() -> turret.openLoop(0), turret),
      ).andThen(new RunCommand(() -> turret.setPositionSetpoint(Rotation2d.fromDegrees(17.221))));
      //.andThen(next);
  @Log
  double turretInput;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    vision.setLED(VisionLEDMode.kOff);
    drivetrain.setDefaultCommand(
      new DriveArcadeOpenLoop(
        driver::getRightTriggerAxis, 
        driver::getLeftX, 
        driver::getLeftTriggerAxis, 
        drivetrain
      )
    );
    flywheel.setDefaultCommand(new RunCommand(flywheel::setFlywheelIdle, flywheel));
    drivetrain.setInverted(true);
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.start(), frontConveyor));
    frontIntake.setDefaultCommand(new RunCommand(() -> frontIntake.setMotorSetpoint(0.0), frontIntake));
    turret.setDefaultCommand(turretCmd);//new RunCommand(turret::stop, turret));
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
    op_up.whenActive(new ConditionalCommand(
      new InstantCommand(frontIntake::raiseIntake, frontIntake), // intake down, so raise it
      new InstantCommand(frontIntake::lowerIntake, frontIntake), // intake up, so lower it
      frontIntake::isExtended)
    );
    op_left.whileActiveOnce(new StartEndCommand(
      frontIntake::setReverse, // inverted, so go forward
      frontIntake::setForward, // forward up, so invert it
      frontIntake
      ).alongWith(new StartEndCommand(
        frontConveyor::setReversed, 
        frontConveyor::setForward, 
        frontConveyor
        )
      ));
    op_right.whenActive(new ConditionalCommand(
      new InstantCommand(backIntake::raiseIntake, backIntake), // intake down, so raise it
      new InstantCommand(backIntake::lowerIntake, backIntake), // intake up, so lower it
      backIntake::isExtended)
    );
    op_down.whileActiveOnce(
      new StartEndCommand(
        backIntake::setReverse, // invert 
        backIntake::setForward, // uninvert
        backIntake
      ).alongWith(new StartEndCommand(
        backConveyor::setReversed, 
        backConveyor::setForward, 
        backConveyor
      )
    ));
    //driver_a.whenActive(new InstantCommand(climber::toggleClimber, climber));
    climber.setDefaultCommand(new RunCommand(() -> climber.setInput(-driver.getRightY() * 0.75), climber));
    driver_b.whileActiveOnce(new StartEndCommand(superstructure::runFeeder, superstructure::stopFeeder, superstructure));
    operator_a.whileActiveOnce(new StartEndCommand(
      () -> flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelHighRPM), 
      () -> flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelDumpRPM), 
      flywheel
    ));

    operator_x.whileActiveOnce(
      new StartEndCommand(
        () -> superstructure.setShooterMode(ShooterMode.DUMP), 
        () -> superstructure.setShooterMode(ShooterMode.DISABLED), 
        superstructure
      )
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return lowAuto;
  }
}
