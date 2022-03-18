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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.commands.FollowTrajectory;
import frc.robot.lib.OI.CommandXboxController;
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
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret, climber, shooterModeUpdater, drivetrain);
  Field2d field;
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  Trigger retractClimb = new Trigger();
  Trigger flyWheelAtSetpoint = new Trigger();
  @Log(tabName = "SmartDashboard", name = "Goal Selector")
  SendableChooser<String> goalSelector = new SendableChooser<>();
  @Log(tabName = "SmartDashboard", name = "Distance Selector")
  SendableChooser<Integer> distanceSelector = new SendableChooser<>();

  CommandXboxController driverCmd = new CommandXboxController(0);
  CommandXboxController operatorCmd = new CommandXboxController(1);
  public boolean feederRunning;
  Trigger turretLocked = new Trigger(turret::atSetpoint);
  Trigger flywheelReady = new Trigger(flywheel::atSetpoint);

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
    .andThen(new WaitUntilCommand(turret::getHomed))
    .andThen(() -> superstructure.setShooterMode(ShooterMode.DUMP))
    .andThen(new InstantCommand(() -> turret.setPositionSetpoint(Rotation2d.fromDegrees(TurretConstants.dumpAngle)), turret))
    .andThen(new RunCommand(() -> {}, turret));

  @Log
  double turretInput;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //vision.setLED(VisionLEDMode.kOff);
    drivetrain.setDefaultCommand(
      new DriveArcadeOpenLoop(
        driver::getRightTriggerAxis, 
        driver::getLeftX, 
        driver::getLeftTriggerAxis, 
        drivetrain
      )
    );
    drivetrain.setInverted(true);
    turret.setDefaultCommand(turretCmd);//new RunCommand(turret::stop, turret));
    // Configure the button bindings
    configureButtonBindings();
    goalSelector.addOption("High", "High");
    goalSelector.addOption("Low", "Low");
    distanceSelector.addOption("Tarmac Edge", 1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver
    driverCmd.rightBumper().whenActive(new InstantCommand(drivetrain::toggleGear, drivetrain));
    driverCmd.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP), superstructure));
    driverCmd.b().or(operatorCmd.b()).whileActiveOnce(
      new ConditionalCommand(
        new InstantCommand(frontConveyor::setReversed)
        .andThen(new WaitCommand(0.5))
        .andThen(new StartEndCommand(
          superstructure::runFeeder, 
          superstructure::stopFeeder, 
          superstructure
        ).alongWith(new InstantCommand(frontConveyor::setForward, frontConveyor))
        ), 
      new InstantCommand(backConveyor::setReversed)
        .andThen(new WaitCommand(0.5))
        .andThen(new StartEndCommand(
          superstructure::runFeeder, 
          superstructure::stopFeeder, 
          superstructure
        ).alongWith(new InstantCommand(backConveyor::setForward, backConveyor))), 
      superstructure::getSeesawFront
    ));
    driverCmd.pov.up().whenActive(new InstantCommand(() -> flywheel.setFlywheelSpeed(flywheel.getFlywheelSpeed()-10), flywheel));
    driverCmd.pov.down().whenActive(new InstantCommand(() -> flywheel.setFlywheelSpeed(flywheel.getFlywheelSpeed()+10), flywheel));
    driverCmd.rightStick().debounce(0.5).whenActive(
      new InstantCommand(climber::toggleClimber, climber)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> climber.setEnabled(true))
    );
    // operator
    operatorCmd.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    operatorCmd.pov.up().whenActive(new ConditionalCommand(
      new InstantCommand(frontIntake::raiseIntake, frontIntake), // intake down, so raise it
      new InstantCommand(frontIntake::lowerIntake, frontIntake), // intake up, so lower it
      frontIntake::isExtended)
    );
    operatorCmd.pov.left().whileActiveOnce(new StartEndCommand(
      frontIntake::setReverse, // inverted, so go forward
      frontIntake::setForward, // forward up, so invert it
      frontIntake
      ).alongWith(new StartEndCommand(
        frontConveyor::setReversed, 
        frontConveyor::setForward, 
        frontConveyor
        )
      )
    );
    operatorCmd.pov.right().whenActive(new ConditionalCommand(
      new InstantCommand(backIntake::raiseIntake, backIntake), // intake down, so raise it
      new InstantCommand(backIntake::lowerIntake, backIntake), // intake up, so lower it
      backIntake::isExtended)
    );
    operatorCmd.pov.down().whileActiveOnce(
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
    operatorCmd.rightBumper().whenActive(new InstantCommand(flywheel::incrementOffset));
    operatorCmd.leftBumper().whenActive(new InstantCommand(flywheel::decrementOffset));
    //driverCmd.a().whenActive(new InstantCommand(climber::toggleClimber, climber));
    climber.setDefaultCommand(new RunCommand(() -> climber.setInput(-driver.getRightY() * 0.75), climber));
    operatorCmd.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE)));
    operatorCmd.b().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DISABLED)));
    operatorCmd.x().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP)));

    
    turretLocked.and(flywheelReady).whileActiveOnce(
      new StartEndCommand(
        () -> operatorCmd.setRumble(RumbleType.kLeftRumble, 0.5),
        () -> operatorCmd.setRumble(RumbleType.kRightRumble, 0.5), 
        vision // vision never has any commands, so this is effectively a no-op
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return 
      new FollowTrajectory(drivetrain, distanceSelector.getSelected())
      .andThen(drivetrain::stop)
      .andThen(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure))
      .andThen(new WaitUntilCommand(flywheelReady))
      .andThen(
        new StartEndCommand(
          superstructure::runFeeder, 
          superstructure::stopFeeder, 
          superstructure
        )
      ).withTimeout(5);
  }
}
