// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Special Thanks To:
// 832 for Superstructure assistance
// 449 for Oblarg
// 3512 for Tyler Veness (and feedforwards/sysid)
// 4390 and 4272 for picking us at Kokomo

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.TurretStartup;
import frc.robot.commands.auto.HighGoalOutsideTarmacTimeBased;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Superstructure.ShooterMode;

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
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, vision);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.backDriverCANId, vision);
  Intake frontIntake = new Intake(IntakeConstants.intakeACANId, IntakeConstants.frontIntakeForwardChannel, IntakeConstants.frontIntakeReverseChannel);
  Intake backIntake = new Intake(IntakeConstants.intakeBCANId, IntakeConstants.backIntakeForwardChannel, IntakeConstants.backIntakeReverseChannel);
  Consumer<ShooterMode> shooterModeUpdater = (ShooterMode mode) -> {
    flywheel.setShooterMode(mode);
    turret.setShooterMode(mode);
  };
  Superstructure superstructure = new Superstructure(flywheel, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret, climber, shooterModeUpdater, drivetrain);

  //@Log(tabName = "SmartDashboard", name = "Time Selector")
  SendableChooser<Command> autoSelector = new SendableChooser<>();
  
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  Trigger turretLocked = new Trigger(turret::atSetpoint);
  Trigger flywheelReady = new Trigger(flywheel::atSetpoint);
  Trigger operatorLeftTrigger = new Trigger(() -> operator.getLeftTriggerAxis() < 0.05);
  Trigger operatorRightTrigger = new Trigger(() -> operator.getRightTriggerAxis() < 0.05);
  Trigger robotLinedUp = new Trigger(vision::getAligned);
  Trigger defenseMode = new Trigger(() -> superstructure.getShooterMode() == ShooterMode.DISABLED);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driver.getRightTriggerAxis() + -driver.getLeftTriggerAxis(), -driver.getLeftX()), drivetrain));
    drivetrain.setInverted(true);
    turret.setDefaultCommand(new TurretStartup(superstructure, turret));//new RunCommand(turret::stop, turret));
    // Configure the button bindings
    configureButtonBindings();
    autoSelector.addOption("Normal", new HighGoalOutsideTarmacTimeBased(superstructure, drivetrain, backIntake, backConveyor));
    superstructure.setShooterMode(ShooterMode.DISABLED);
    superstructure.stopFeeder();
    frontConveyor.setName("FrontConveyor");
    backConveyor.setName("BackConveyor");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver
    driver.rightBumper().onTrue(new InstantCommand(drivetrain::toggleGear, drivetrain));
    driver.b().whileTrue(superstructure.shoot(true));
    driver.x().onTrue(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    driver.rightStick().debounce(0.5).onTrue(
      new InstantCommand(climber::toggleClimber, climber)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> climber.setEnabled(true))
    );
    driver.start().onTrue(new InstantCommand(climber::enable, climber));
    climber.setDefaultCommand(new RunCommand(() -> climber.setInput(-driver.getRightY() * 0.75), climber));
    // operator
    operator.a().onTrue(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    operator.leftBumper().onTrue(new ConditionalCommand(
      new InstantCommand(frontIntake::raiseIntake, frontIntake).andThen(new InstantCommand(frontConveyor::stop)), // intake down, so raise it
      new InstantCommand(frontIntake::lowerIntake, frontIntake).andThen(new InstantCommand(frontConveyor::start)), // intake up, so lower it
      frontIntake::isExtended)
    );
    operator.rightBumper().onTrue(new ConditionalCommand(
      new InstantCommand(backIntake::raiseIntake, backIntake).andThen(new InstantCommand(backConveyor::stop)), // intake down, so raise it
      new InstantCommand(backIntake::lowerIntake, backIntake).andThen(new InstantCommand(backConveyor::start)), // intake up, so lower it
      backIntake::isExtended)
    );
    operator.x().onTrue(new InstantCommand(() -> superstructure.setShooterMode(ShooterMode.DISABLED)));
    operator.b().whileTrue(superstructure.rejectCargo());

    // triggers
    robotLinedUp.and(flywheelReady).whileTrue(
      new StartEndCommand(
        () -> { 
          driver.getHID().setRumble(RumbleType.kLeftRumble, 0.5);
          driver.getHID().setRumble(RumbleType.kRightRumble, 0.5); 
        },
        () -> { 
          driver.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
          driver.getHID().setRumble(RumbleType.kRightRumble, 0.0); 
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
    return new HighGoalOutsideTarmacTimeBased(superstructure, drivetrain, backIntake, backConveyor);
  }
  //TODO: use suppliers instead of setFlywheelX
}
