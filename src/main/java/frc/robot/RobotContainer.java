// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Special Thanks To:
// 832 for Superstructure assistance
// 449 for Oblarg
// 3512 for Tyler Veness (and feedforwards/sysid)
// 4390 and 4272 for picking us at Kokomo

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretStartup;
import frc.robot.commands.auto.HighGoalOutsideTarmacTimeBased;
//import frc.robot.commands.FollowTrajectory;
import frc.robot.lib.OI.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
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
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, 1);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.backDriverCANId, 2);
  Intake frontIntake = new Intake(intakeACANId, frontIntakeForwardChannel, frontIntakeReverseChannel);
  Intake backIntake = new Intake(intakeBCANId, backIntakeForwardChannel, backIntakeReverseChannel);
  Consumer<ShooterMode> shooterModeUpdater = (ShooterMode mode) -> {
    flywheel.setShooterMode(mode);
    turret.setShooterMode(mode);
  };
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret, climber, shooterModeUpdater, drivetrain);
  Field2d field;

  Trigger retractClimb = new Trigger();
  Trigger robotLinedUp = new Trigger(vision::getAligned);
  Trigger defenseMode = new Trigger(() -> superstructure.getShooterMode() == ShooterMode.DISABLED);
  @Log(tabName = "SmartDashboard", name = "Time Selector")
  SendableChooser<Double> timeSelector = new SendableChooser<>();
  @Log(tabName = "SmartDashboard", name = "Distance Selector")
  SendableChooser<Double> distanceSelector = new SendableChooser<>();

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  public boolean feederRunning;
  Trigger turretLocked = new Trigger(turret::atSetpoint);
  Trigger flywheelReady = new Trigger(flywheel::atSetpoint);

  Trigger operatorLeftTrigger = new Trigger(() -> operator.getLeftTriggerAxis() < 0.05);
  Trigger operatorRightTrigger = new Trigger(() -> operator.getRightTriggerAxis() < 0.05);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driver.getRightTriggerAxis() + -driver.getLeftTriggerAxis(), -driver.getLeftX()), drivetrain));
    drivetrain.setInverted(true);
    turret.setDefaultCommand(new TurretStartup(superstructure, turret));//new RunCommand(turret::stop, turret));
    // Configure the button bindings
    configureButtonBindings();
    timeSelector.addOption("Wall", 0.85);
    timeSelector.addOption("Long", 0.95);
    distanceSelector.addOption("Tarmac Edge", Units.inchesToMeters(42));
    superstructure.setShooterMode(ShooterMode.DISABLED);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver
    driver.rightBumper().whenActive(new InstantCommand(drivetrain::toggleGear, drivetrain));
    //driver.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DUMP_LOW), superstructure));
    /*driver.b().or(operator.b()).and(defenseMode.negate()).whileActiveOnce(
      new ConditionalCommand(
        new TurretTrack(turret, vision)
        .andThen(new WaitUntilCommand(() -> vision.hasTargets() && turret.atSetpoint()))
        .andThen(new StartEndCommand(
          superstructure::runFeeder, 
          superstructure::stopFeeder, 
          superstructure
        )
      ), 
      new InstantCommand(backConveyor::setReversed)
        .andThen(new WaitCommand(0.5))
        .andThen(new StartEndCommand(
          superstructure::runFeeder, 
          superstructure::stopFeeder, 
          superstructure
        ).alongWith(new InstantCommand(backConveyor::setForward, backConveyor))), 
      superstructure::getSeesawFront
    ));*/
    driver.x().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    driver.rightStick().debounce(0.5).whenActive(
      new InstantCommand(climber::toggleClimber, climber)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> climber.setEnabled(true))
    );
    // operator
    operator.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    
    operator.leftBumper().whenActive(new ConditionalCommand(
      new InstantCommand(frontIntake::raiseIntake, frontIntake), // intake down, so raise it
      new InstantCommand(frontIntake::lowerIntake, frontIntake), // intake up, so lower it
      frontIntake::isExtended)
    );
    /*operatorLeftTrigger.whileActiveOnce(new StartEndCommand(
      frontIntake::setForward, // inverted, so go forward
      frontIntake::setReverse, // forward up, so invert it
      frontIntake
      ).alongWith(new StartEndCommand(
        frontConveyor::setReversed, 
        frontConveyor::setForward, 
        frontConveyor
        )
      )
    );*/
    operator.rightBumper().whenActive(new ConditionalCommand(
      new InstantCommand(backIntake::raiseIntake, backIntake), // intake down, so raise it
      new InstantCommand(backIntake::lowerIntake, backIntake), // intake up, so lower it
      backIntake::isExtended)
    );
    /*operatorRightTrigger.whileActiveOnce(
      new StartEndCommand(
        backIntake::setForward, // invert 
        backIntake::setReverse, // uninvert
        backIntake
      ).alongWith(new StartEndCommand(
        backConveyor::setForward, 
        backConveyor::setReversed, 
        backConveyor
      )
    ));*/
    /*operatorCmd.rightBumper().whenActive(new InstantCommand(flywheel::incrementOffset));
    operatorCmd.leftBumper().whenActive(new InstantCommand(flywheel::decrementOffset));*/
    driver.start().whenActive(new InstantCommand(climber::enable, climber));
    climber.setDefaultCommand(new RunCommand(() -> climber.setInput(-driver.getRightY() * 0.75), climber));
    operator.a().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE)));
    operator.x().whenActive(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DISABLED)));

    
    robotLinedUp.and(flywheelReady).whileActiveOnce(
      new StartEndCommand(
        () -> { 
          driver.setRumble(RumbleType.kLeftRumble, 0.5);
          driver.setRumble(RumbleType.kRightRumble, 0.5); 
        },
        () -> { 
          driver.setRumble(RumbleType.kLeftRumble, 0.0);
          driver.setRumble(RumbleType.kRightRumble, 0.0); 
        }
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new HighGoalOutsideTarmacTimeBased(superstructure, drivetrain, backIntake);
  }
}
