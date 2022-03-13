// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.GoalTrack;
//import frc.robot.lib.PicoColorSensor;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Coordinates all subsystems involving cargo
 * Drivers command Superstructure, which passes it on to the subsystems
 * It makes sure two subsystems are ready for handoff before initiating it
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase {
  // Intake
  private final Intake frontIntake;
  private final Intake backIntake;
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor frontConveyor;
  private final Conveyor backConveyor;
  
  //private final PicoColorSensor colorSensor;
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
  @Log.BooleanBox(methodName = "get")
  public Trigger robotLinedUp;
  // Intakes
  // Flywheel
  public Trigger flywheelEnabled;
  public Trigger turretEnabled;
  //TODO: Log conveyor/intake states in superstructure separately to avoid collisions
  //TODO: Create Driver Dashboard
  //TODO: make sure back intake/conveyor operates normally
  //TODO: program merger
  ShooterMode mode;
  @Config
  double flywheelRPM = 0.0;
  public boolean isForward;
  public double feederSpeedRunning = 0.75;
  public double feederSpeedStopped = 0.0; 
  WPI_TalonFX feederA;
  WPI_TalonFX feederB;
  DoubleSolenoid merger;
  GoalTrack goalTrack;

  @Log.BooleanBox(name = "Active Conveyor", colorWhenTrue = "#d6e810", colorWhenFalse = "#1861d6")
  boolean seesawFront = true;

  ParallelRaceGroup fullAuto;
  ParallelRaceGroup manualShoot;
  ParallelRaceGroup disabled;
  ParallelRaceGroup dump;
  ParallelRaceGroup testing;
  Consumer<ShooterMode> shooterModeUpdater;

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
    
    goalTrack = new GoalTrack(0, new Translation2d());
    //colorSensor = new PicoColorSensor();
    mode = ShooterMode.DISABLED;
    feederA = new WPI_TalonFX(10);
    feederA.setInverted(true);
    feederB = new WPI_TalonFX(11);

    merger = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ConveyorConstants.seesawForwardPCMId, Constants.ConveyorConstants.seesawReversePCMId);

    flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setFlywheelSpeed(0.0), flywheel));
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.start(), frontConveyor));
    turret.setDefaultCommand(new RunCommand(()-> turret.stop(), turret));
    backConveyor.setDefaultCommand(new RunCommand(backConveyor::start, backConveyor));

    
    shooterReady = new Trigger(this::getShooterReady);
    seesawReady = new Trigger();
    frontConveyorFull = new Trigger(frontConveyor::isBallPresent);
    backConveyorFull = new Trigger(backConveyor::isBallPresent);
    flywheelEnabled = new Trigger(flywheel::getActive);
    turretEnabled = new Trigger(turret::getActive);
    flyWheelAtSetpoint = new Trigger(()-> {return !flywheel.atSetpoint();});
    robotLinedUp = new Trigger(() -> vision.getBestTarget().getYaw() < 1);    

    setupConveyorCommands();
    setupShooterCommands();
  }

  private void setupConveyorCommands() {
  }
  
  private void setupShooterCommands() {
    //frontConveyorFull.whenActive(new InstantCommand(this::seesawToFront));
    //backConveyorFull.and(frontConveyorFull.negate()).whenActive(new InstantCommand(this::seesawToRear));

    // set speed
    turretEnabled.whileActiveOnce(new RunCommand(() -> { 
      if (vision.hasTargets()) 
      { 
        flywheel.setFlywheelDistance(vision.getTargetDistance(vision.getBestTarget()));
        turret.setSetpointDegrees(vision.getClosestTarget().getYaw());
      } 
    }, flywheel, turret));
    flywheelEnabled.and(turretEnabled.negate()).whileActiveOnce(new RunCommand(() -> {
      if (vision.hasTargets()) {
        flywheel.setFlywheelDistance(vision.getTargetDistance(vision.getBestTarget()));
      }
    }, flywheel));

    //frontConveyorFull.whileActiveOnce(new StartEndCommand(this::runFeeder, this::stopFeeder, this));
    //vision.setLED(VisionLEDMode.kOff);
    //turret.setDefaultCommand(new RunCommand(() -> { turret.setSetpointDegrees(0); }, turret));

  }

  public void runFeederA() {
    feederA.set(feederSpeedRunning);
    feederA.setNeutralMode(NeutralMode.Coast);
  }
  public void runFeederB() {
    feederB.set(feederSpeedRunning);
    feederB.setNeutralMode(NeutralMode.Coast);
  }
  public void runFeeder() {
    feederA.set(feederSpeedRunning);
    feederB.set(feederSpeedRunning);
    feederA.setNeutralMode(NeutralMode.Coast);
    feederB.setNeutralMode(NeutralMode.Coast);
  }
  
  public void stopFeeder() {
    feederA.set(feederSpeedStopped);
    feederB.set(feederSpeedStopped);
    feederA.setNeutralMode(NeutralMode.Brake);
    feederB.setNeutralMode(NeutralMode.Brake);
  }

  public void seesawToRear() {
    merger.set(Value.kForward);
  }

  public void seesawToFront() {
    merger.set(Value.kReverse);
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
      case FULL_AUTO:
        NetworkTableInstance.getDefault().setUpdateRate(0.01);
        vision.setLED(VisionLEDMode.kOn);
        break;
      case DUMP:
        vision.setLED(VisionLEDMode.kOn);
        break;
      case DISABLED:
        NetworkTableInstance.getDefault().setUpdateRate(0.1);
        vision.setLED(VisionLEDMode.kOff);
        break;
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
    FULL_AUTO, // turret and flywheel track the goal, ball is fired if present as soon as shooter is ready
    MANUAL_FIRE, // turret and flywheel track the goal, ball is fired on operator command if present
    DUMP, // Turret locks to dead ahead but flywheel is set to minimum
    HOMING,
    DISABLED // turret and flywheel do not move, shooting is impossible
  }
  // setShooterMode method here
  // subsystems check shooter mode, act accordingly

  public void addVisionUpdate(double timestamp, PhotonTrackedTarget target) {
    //counter clock wise is positivie thats why target yaw is inverted
    //pretty sure x is positive forward, y is positive left
        
    double yaw = -target.getYaw();
  
    double distance = vision.getTargetDistance(target);
    Rotation2d angle = turret.getCurrentPosition().plus(Rotation2d.fromDegrees(yaw)).plus(drivetrain.getGyroAngle());
    Translation2d field_to_goal=new Translation2d(distance * angle.getCos(), distance * angle.getSin());
    goalTrack.tryUpdate(timestamp, field_to_goal);
    // System.out.println("time: "+timestamp+ " x: "+field_to_goal.getX()+" y: "+field_to_goal.getY());
    

}

  public void updateVision() {
    if (vision.hasTargets()) {
      addVisionUpdate(Timer.getFPGATimestamp(), vision.getBestTarget());
    }
  }

  @Override
  public void periodic() {
    Translation2d smoothedPos = goalTrack.getSmoothedPosition();
    //NTHelper.setDouble("smooth_pos_deg", value);
    switch (mode) {
      case MANUAL_FIRE:
      case FULL_AUTO:
        turret.setPositionSetpoint(new Rotation2d(smoothedPos.getX(), smoothedPos.getY()).minus(drivetrain.getGyroAngle()));
        break;
      case DUMP:
      case DISABLED:
      default:
        break;
    }
    
  }
  
}
