// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision;
import frc.robot.commands.TurretStartup;
import frc.robot.commands.TurretTrack;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Coordinates all subsystems involving cargo<p>
 * Drivers command Superstructure, which passes it on to the subsystems<p>
 * It makes sure two subsystems are ready for handoff before initiating it<p>
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase implements Loggable {
  // Intake
  private final Intake frontIntake;
  private final Intake backIntake;
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor frontConveyor;
  private final Conveyor backConveyor;
  
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  private final Turret turret;
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
  @Log.BooleanBox(name = "Robot Aligned", methodName = "get", tabName = "SmartDashboard")
  public Trigger robotLinedUp;
  // Intakes
  // Flywheel
  public Trigger flywheelEnabled;
  public Trigger turretEnabled;

  
  //Driver Dashboard
  // active intake 
  // camera
  // turret position DONE
  // low/high gear DONE
  // robot lined up rumble DONE
  // flywheel at speed DONE
  
  @Log.BooleanBox(tabName = "SmartDashboard", name = "Turret Position", colorWhenTrue = "yellow", colorWhenFalse = "blue")
  boolean turretAtFront = true;
  ShooterMode mode;
  @Config
  double flywheelRPM = 0.0;
  public boolean isForward;
  @Log
  public double frontFeederSpeedRunning = 0.5;
  @Log
  public double rearFeederSpeedRunning = 0.5;
  public double feederSpeedStopped = 0.0; 
  WPI_TalonFX frontFeeder;
  WPI_TalonFX rearFeeder;
  @Log(tabName = "SmartDashboard", name = "Distance to Hub")
  double distance;

  @Log.BooleanBox(name = "Seesaw Position", colorWhenTrue = "yellow", colorWhenFalse = "blue", tabName = "SmartDashboard")
  boolean seesawFront = true;

  Consumer<ShooterMode> shooterModeUpdater;
  BooleanSupplier visionHasTarget;
  DoubleSupplier targetYaw;
  MedianFilter yawSmooth = new MedianFilter(10);

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor, Intake frontIntake,  Intake backIntake, Vision vision, Turret turret, Climber climber, Consumer<ShooterMode> shooterModeUpdater, Drivetrain drivetrain) {
    this.flywheel = flywheel;
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
    this.frontIntake = frontIntake;
    this.backIntake = backIntake;
    this.turret = turret;
    this.vision = vision;
    this.climber = climber;
    this.drivetrain = drivetrain;
    this.shooterModeUpdater = shooterModeUpdater;
    
    mode = ShooterMode.DISABLED;
    frontFeeder = new WPI_TalonFX(10);
    frontFeeder.setInverted(true);
    rearFeeder = new WPI_TalonFX(11);
    frontFeeder.enableVoltageCompensation(true);
    frontFeeder.configVoltageCompSaturation(12);
    rearFeeder.enableVoltageCompensation(true);
    rearFeeder.configVoltageCompSaturation(12);
    frontFeeder.configOpenloopRamp(0.1);
    rearFeeder.configOpenloopRamp(0.1);

    //frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    turret.setDefaultCommand(new RunCommand(()-> turret.stop(), turret));
    //backConveyor.setDefaultCommand(new RunCommand(backConveyor::stop, backConveyor));

    
    shooterReady = new Trigger(this::getShooterReady);
    robotLinedUp = new Trigger(vision::getAligned);
    flywheelEnabled = new Trigger(flywheel::getActive);
    turretEnabled = new Trigger(turret::getActive);
    flyWheelAtSetpoint = new Trigger(()-> {return !flywheel.atSetpoint();});

    targetYaw = this::getTargetYaw;
    setDefaultCommand(new RunCommand(this::stopFeeder, this));

  }
  
//@Log(tabName = "team6498")
  private double getTargetYaw() {
    if (vision.hasTargets()) {
      return yawSmooth.calculate(vision.getBestTarget().getYaw());
    } else { return 0; }
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
    frontFeeder.set(-1);
    rearFeeder.set(-1);
    frontFeeder.setNeutralMode(NeutralMode.Coast);
    rearFeeder.setNeutralMode(NeutralMode.Coast);
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
    frontFeeder.set(frontFeederSpeedRunning);
    rearFeeder.set(rearFeederSpeedRunning);
    frontFeeder.setNeutralMode(NeutralMode.Coast);
    rearFeeder.setNeutralMode(NeutralMode.Coast);
  }
  
  public void stopFeeder() {
    frontFeeder.set(feederSpeedStopped);
    rearFeeder.set(feederSpeedStopped);
    frontFeeder.setNeutralMode(NeutralMode.Brake);
    rearFeeder.setNeutralMode(NeutralMode.Brake);
  }

  public Color getAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    switch (alliance) {
      case Red:
        return Color.kRed;
      case Blue:
        return Color.kBlue;
      case Invalid:
      default:
        return Color.kBrown;
    }
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
        NetworkTableInstance.getDefault().setUpdateRate(0.01);
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
  public void periodic() {
    if (mode == ShooterMode.MANUAL_FIRE && turret.getDefaultCommand() instanceof TurretStartup) {
      turret.setDefaultCommand(new TurretTrack(turret, targetYaw));
    }
    if (turret.getCurrentPosition().getDegrees() > -10) { // 0, front
      turretAtFront = true;
    } else {
      turretAtFront = false;
    }
    if (vision.hasTargets()) {
      flywheel.setFlywheelDistance(vision.getTargetDistance(vision.getBestTarget()));
    }
  } 
}
