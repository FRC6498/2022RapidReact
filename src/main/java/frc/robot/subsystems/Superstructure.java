// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;


import org.photonvision.common.hardware.VisionLEDMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision;
import monologue.Logged;
import monologue.Annotations.Log;

/**
 * Coordinates all subsystems involving cargo<p>
 * Drivers command Superstructure, which passes it on to the subsystems<p>
 * It makes sure two subsystems are ready for handoff before initiating it<p>
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase implements Logged {
  // Intake
  private final Intake frontIntake;
  private final Intake backIntake;
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor frontConveyor;
  private final Conveyor backConveyor;
  
  // Feeder
  //climber
  public final Climber climber;
  public final Drivetrain drivetrain;
  // Triggers
  // Superstructure
  public Trigger shooterReady;
  public Trigger seesawReady;
  // Conveyors
  public Trigger frontConveyorFull;
  public Trigger backConveyorFull;
  public Trigger flyWheelAtSetpoint;
  @Log //.BooleanBox(name = "Robot Aligned", methodName = "get", tabName = "SmartDashboard")
  public Trigger robotLinedUp;
  // Intakes
  // Flywheel
  public Trigger flywheelEnabled;
  public Trigger turretEnabled;

  
  //TODO: Create Driver Dashboard
  // active intake 
  // camera
  // turret position DONE
  // low/high gear DONE
  // robot lined up rumble DONE
  // flywheel at speed DONE
  @Log //.BooleanBox(tabName = "SmartDashboard", name = "Turret Position", colorWhenTrue = "yellow", colorWhenFalse = "blue")
  private Trigger turretAtFront;
  ShooterMode mode;
  //@Config
  double flywheelRPM = 0.0;
  public boolean isForward;
  public double feederSpeedRunning = 0.5;
  public double feederSpeedStopped = 0.0; 
  TalonFX frontFeeder;
  TalonFX rearFeeder;
  @Log //(tabName = "SmartDashboard", name = "Distance to Hub")
  double distanceToHub;

  Consumer<ShooterMode> shooterModeUpdater;
  BooleanSupplier visionHasTarget;
  DoubleSupplier targetYaw;

  DutyCycleOut feederPercent = new DutyCycleOut(0);

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor, Intake frontIntake,  Intake backIntake, Vision vision, Turret turret, Climber climber, Consumer<ShooterMode> shooterModeUpdater, Drivetrain drivetrain) {
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
    this.frontIntake = frontIntake;
    this.backIntake = backIntake;
    this.vision = vision;
    this.climber = climber;
    this.drivetrain = drivetrain;
    this.shooterModeUpdater = shooterModeUpdater;
    
    frontFeeder = new TalonFX(10);
    frontFeeder.setInverted(true);
    rearFeeder = new TalonFX(11);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    frontFeeder.getConfigurator().apply(config);
    rearFeeder.getConfigurator().apply(config);
    rearFeeder.setControl(new Follower(frontFeeder.getDeviceID(), true));
    
    shooterReady = new Trigger(this::getShooterReady);
    robotLinedUp = new Trigger(vision::getAligned);
    flywheelEnabled = new Trigger(flywheel::getActive);
    turretEnabled = new Trigger(turret::getActive);
    flyWheelAtSetpoint = new Trigger(()-> {return !flywheel.atSetpoint();});
    turretAtFront = new Trigger(() -> turret.getCurrentPosition().getDegrees() > -10);
  }

  public Command rejectCargo() {
    return Commands.sequence(
      runOnce(() -> setShooterMode(ShooterMode.REJECT)),
      shoot(false),
      runOnce(() -> setShooterMode(ShooterMode.MANUAL_FIRE))
    );
  }

  public Command shoot(boolean useTurret) {
    return Commands.sequence(
      Commands.waitUntil(() -> flyWheelAtSetpoint.getAsBoolean() && robotLinedUp.getAsBoolean() || !useTurret),
      runOnce(this::runFrontConveyorReverse),
      runOnce(this::runRearConveyorReverse),
      Commands.waitSeconds(0.4),
      runOnce(this::runFeederReverse),
      Commands.waitSeconds(0.15),
      runOnce(this::runFeeder),
      Commands.waitSeconds(0.1),
      runOnce(this::runFrontConveyor),
      runOnce(this::runRearConveyor),
      Commands.idle(this)
    ).finallyDo(() -> {
      stopFrontConveyor();
      stopRearConveyor();
      stopFeeder();
    });
  }

  public void runFrontConveyor() {
    frontConveyor.start();
  }

  public void runFrontConveyorSlow() {
    frontConveyor.start(0.5);
  }

  public void runFrontConveyorReverse() {
    frontConveyor.start(-0.1);
  }

  public void runRearConveyorReverse() {
    backConveyor.start(-0.1);
  }

  public void runFeederReverse() {
    frontFeeder.setControl(feederPercent.withOutput(-1));
  }

  public void runRearConveyorSlow() {
    backConveyor.start(0.5);
  }

  public void runRearConveyor() {
    backConveyor.start();
  }

  public void stopFrontConveyor() {
    frontConveyor.stop();
  }

  public void stopRearConveyor() {
    backConveyor.stop();
  }

  public void runFeeder() {
    frontFeeder.setControl(feederPercent.withOutput(feederSpeedRunning));
  }
  
  public void stopFeeder() {
    frontFeeder.setControl(feederPercent.withOutput(0));
  }

  public Color getAllianceColor() {
    if (DriverStation.getAlliance().isPresent()) {
      var alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        return Color.kBlue;
      } else {
        return Color.kRed;
      }
    }
    return Color.kBlack;
  }

  public void getBallColors() {
    //backConveyor.setCargoColor(colorSensor.getColor(backConveyor.colorSensorId));
    //frontConveyor.setCargoColor(colorSensor.getColor(frontConveyor.colorSensorId));
  } 

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
    shooterModeUpdater.accept(mode);
    switch (mode) {
      case MANUAL_FIRE:
        vision.setLED(VisionLEDMode.kOn);
        vision.setDriverMode(false);
        break;
      case DISABLED:
        stopFeeder();
        frontIntake.raiseIntake();
        backIntake.raiseIntake();
        frontConveyor.stop();
        backConveyor.stop();
      default:
        break;
    }
  }

  public ShooterMode getShooterMode() {
    return mode;
  }

  public boolean getShooterReady() {
    return false;
    //return flywheel.atSetpoint() && turret.atSetpoint();
  }
  

  public enum ShooterMode {
    MANUAL_FIRE, // turret and flywheel track the goal, ball is fired on operator command if present
    REJECT, // Turret locks to dead ahead but flywheel is set to minimum
    //DUMP_HIGH,
    //SEARCHING, 
    HOMING, //
    TUNING, // Turret Disabled, flywheel speed settable
    DISABLED // turret and flywheel do not move, shooting is impossible
  }
  // setShooterMode method here
  // subsystems check shooter mode, act accordingly

  @Override
  public void periodic() {} 
}
