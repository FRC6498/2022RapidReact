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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Superstructure.ShooterMode;
import monologue.Logged;
import monologue.Monologue;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Consumer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  Drivetrain drivetrain = new Drivetrain();
  Flywheel flywheel = new Flywheel();
  Vision vision = new Vision();
  Turret turret = new Turret(vision::getTargetYaw);
  Climber climber = new Climber();
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, false);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.backDriverCANId, true);
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
    Monologue.setupMonologue(this, "RobotContainer", false, false);
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driver.getRightTriggerAxis() + -driver.getLeftTriggerAxis(), -driver.getLeftX()), drivetrain));
    drivetrain.setInverted(true);
    turret.setDefaultCommand(turret.home().andThen(turret.track()));//new RunCommand(turret::stop, turret));
    // Configure the button bindings
    configureButtonBindings();
    autoSelector.addOption("Normal", Autos.highGoalOutsideTarmacTimeBased(backIntake, backConveyor, drivetrain, superstructure));
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
    driver.rightBumper().onTrue(runOnce(drivetrain::toggleGear, drivetrain));
    driver.b().whileTrue(superstructure.shoot(true));
    driver.x().onTrue(runOnce(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    driver.rightStick().debounce(0.5).onTrue(
      runOnce(climber::unlockClimber, climber)
      .andThen(waitSeconds(0.5))
      .andThen(climber.manualDrive(driver::getLeftY))
    );
    driver.start().onTrue(climber.manualDrive(driver::getLeftY));
    // operator
    operator.a().onTrue(runOnce(() -> superstructure.setShooterMode(ShooterMode.MANUAL_FIRE), superstructure));
    operator.leftBumper().onTrue(new ConditionalCommand(
      runOnce(frontIntake::raiseIntake, frontIntake).andThen(runOnce(frontConveyor::stop)), // intake down, so raise it
      runOnce(frontIntake::lowerIntake, frontIntake).andThen(runOnce(frontConveyor::start)), // intake up, so lower it
      frontIntake::isExtended)
    );
    operator.rightBumper().onTrue(new ConditionalCommand(
      runOnce(backIntake::raiseIntake, backIntake).andThen(runOnce(backConveyor::stop)), // intake down, so raise it
      runOnce(backIntake::lowerIntake, backIntake).andThen(runOnce(backConveyor::start)), // intake up, so lower it
      backIntake::isExtended)
    );
    operator.x().onTrue(runOnce(() -> superstructure.setShooterMode(ShooterMode.DISABLED)));
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
    return Autos.highGoalOutsideTarmacTimeBased(backIntake, backConveyor, drivetrain, superstructure);
  }
  //TODO: use suppliers instead of setFlywheelX
}
