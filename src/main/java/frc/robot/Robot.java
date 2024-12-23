// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //UsbCamera frontCamera;
  Drivetrain drivetrain = new Drivetrain();
  Shooter shooter = new Shooter();
  Vision vision = new Vision();
  Turret turret = new Turret(vision::getTargetYaw);
  Climber climber = new Climber();
  Conveyor frontConveyor = new Conveyor(Constants.ConveyorConstants.frontDriverCANId, false);
  Conveyor backConveyor = new Conveyor(Constants.ConveyorConstants.backDriverCANId, true);
  Intake frontIntake = new Intake(IntakeConstants.intakeACANId, IntakeConstants.frontIntakeForwardChannel, IntakeConstants.frontIntakeReverseChannel);
  Intake backIntake = new Intake(IntakeConstants.intakeBCANId, IntakeConstants.backIntakeForwardChannel, IntakeConstants.backIntakeReverseChannel);
  Superstructure superstructure = new Superstructure(shooter, frontConveyor, backConveyor, frontIntake, backIntake, vision, turret, climber, drivetrain);;

  SendableChooser<Command> autoSelector = new SendableChooser<>();
  
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  Trigger turretLocked = new Trigger(turret::atSetpoint);
  Trigger flywheelReady = new Trigger(shooter::atSetpoint);
  Trigger operatorLeftTrigger = new Trigger(() -> operator.getLeftTriggerAxis() < 0.05);
  Trigger operatorRightTrigger = new Trigger(() -> operator.getRightTriggerAxis() < 0.05);
  Trigger robotLinedUp = new Trigger(vision::getAligned);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //DataLogManager.start();
    Epilogue.configure((config) -> {
      config.backend = EpilogueBackend.multi(new FileBackend(DataLogManager.getLog()), new NTEpilogueBackend(NetworkTableInstance.getDefault()));
    });
    Epilogue.bind(this);
    LiveWindow.disableAllTelemetry();
    
    //frontCamera = CameraServer.startAutomaticCapture();
    //frontCamera.setResolution(320, 240);
    drivetrain.resetSensors();
    addPeriodic(() -> vision.periodic(), 0.02);

    drivetrain.setDefaultCommand(Commands.run(() -> drivetrain.arcadeDrive(driver.getRightTriggerAxis() + -driver.getLeftTriggerAxis(), -driver.getLeftX()), drivetrain));
    drivetrain.setInverted(true);
    turret.setDefaultCommand(turret.home().andThen(turret.track()));
    // Configure the button bindings
    configureButtonBindings();
    autoSelector.addOption("Normal", Autos.highGoalOutsideTarmacTimeBased(backIntake, backConveyor, drivetrain, superstructure));
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
    DriverStation.silenceJoystickConnectionWarning(true);
    // driver
    driver.rightBumper().onTrue(Commands.runOnce(drivetrain::toggleGear, drivetrain));
    driver.b().whileTrue(superstructure.shoot(true));
    driver.x().onTrue(superstructure.manualFire());
    driver.rightStick().debounce(0.5).onTrue(
      Commands.runOnce(climber::unlockClimber, climber)
      .andThen(Commands.waitSeconds(0.5))
      .andThen(climber.manualDrive(driver::getLeftY))
    );
    driver.start().onTrue(climber.manualDrive(driver::getLeftY));

    // operator
    operator.a().onTrue(superstructure.manualFire());
    operator.leftBumper().onTrue(Commands.either(
      Commands.runOnce(frontIntake::raiseIntake, frontIntake).andThen(Commands.runOnce(frontConveyor::stop)), // intake down, so raise it
      Commands.runOnce(frontIntake::lowerIntake, frontIntake).andThen(Commands.runOnce(frontConveyor::start)), // intake up, so lower it
      frontIntake::isExtended)
    );
    operator.rightBumper().onTrue(Commands.either(
      Commands.runOnce(backIntake::raiseIntake, backIntake).andThen(Commands.runOnce(backConveyor::stop)), // intake down, so raise it
      Commands.runOnce(backIntake::lowerIntake, backIntake).andThen(Commands.runOnce(backConveyor::start)), // intake up, so lower it
      backIntake::isExtended)
    );
    operator.x().onTrue(Commands.runOnce(() -> superstructure.safeIdle()));
    operator.b().whileTrue(superstructure.rejectCargo());

    // triggers
    robotLinedUp.and(flywheelReady).whileTrue(
      Commands.runEnd(
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
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    drivetrain.resetSensors();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    drivetrain.resetSensors();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  
   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shooter.manualSpeed();
    //return Autos.highGoalOutsideTarmacTimeBased(backIntake, backConveyor, drivetrain, superstructure);
  }

}
