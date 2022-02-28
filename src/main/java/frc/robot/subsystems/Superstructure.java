// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
//import frc.robot.lib.PicoColorSensor;
import frc.robot.lib.ShotMap;
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
  private final Intake intake;
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor conveyor;
  
  //private final PicoColorSensor colorSensor;
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  private final Turret turret;
  // Feeder
  //climber
  public final Climber climber;
  // Triggers
  // Superstructure
  public Trigger shooterReady;
  public Trigger seesawReady;
  // Conveyors
  public Trigger frontConveyorFull;
  public Trigger frontConveyorBallColorCorrect;
  public Trigger backConveyorFull;
  public Trigger backConveyorBallColorCorrect;
  public Trigger inLowGear;
  public Trigger flyWheelAtSetpoint;
  // Intakes
  // Flywheel
  public Trigger shooterAutoEnabled;
  ShotMap flywheelTable = new ShotMap();
  ShooterMode shooterMode;
  @Config
  double flywheelRPM = 0.0;
  public boolean isForward;
  public double feederSpeedRunning = 0.75;
  public double feederSpeedStopped = 0.0; 
  WPI_TalonFX feederA;
  WPI_TalonFX feederB;

  ParallelRaceGroup fullAuto;
  ParallelRaceGroup manualShoot;
  ParallelRaceGroup disabled;
  ParallelRaceGroup dump;
  ParallelRaceGroup testing;
  

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Intake frontIntake, Vision vision, Turret turret, Climber climber) {
    this.flywheel = flywheel;
    this.conveyor = frontConveyor;
    this.intake = frontIntake;
    this.turret = turret;
    this.vision = vision;
    this.climber = climber;
    
    //colorSensor = new PicoColorSensor();

    feederA = new WPI_TalonFX(10);
    feederA.setInverted(true);
    feederB = new WPI_TalonFX(11);

    //flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setFlywheelIdle(), flywheel));
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    turret.setDefaultCommand(new RunCommand(()-> turret.stop(), turret));


    flywheelTable.put(0, 0.8);
    flywheelTable.put(Units.feetToMeters(9), 1.6);
    flywheelTable.put(Units.inchesToMeters(10*12+4), 1.8);
    shooterReady = new Trigger(this::getShooterReady);
    seesawReady = new Trigger();
    frontConveyorFull = new Trigger(() -> frontConveyor.isBallPresent(false));
    frontConveyorBallColorCorrect = new Trigger(() -> {return frontConveyor.getCargoColor() == this.getAllianceColor(); });
    shooterAutoEnabled = new Trigger(flywheel::getFlywheelActive);
    inLowGear = new Trigger(() -> {return !Drivetrain.isHighGear;});
    flyWheelAtSetpoint = new Trigger(()-> {return !flywheel.atSetpoint();});

    fullAuto = new ParallelCommandGroup(
      new RunCommand(() -> flywheel.setFlywheelSpeed(flywheelTable.getRPM(vision.getTargetDistance(vision.getBestTarget()))), this.flywheel)
    ).withInterrupt(() -> { return this.getShooterMode() != ShooterMode.FULL_AUTO; });

    dump = new ParallelCommandGroup(
      new RunCommand(() -> this.flywheel.setFlywheelSpeed(Constants.ShooterConstants.flywheelDumpRPM), this.flywheel)
    ).withInterrupt(() -> { return this.getShooterMode() != ShooterMode.DUMP;} );
    
    manualShoot = new ParallelRaceGroup(
      new RunCommand(() -> flywheel.setFlywheelSpeed(flywheelTable.getRPM(vision.getTargetDistance(vision.getBestTarget()))), flywheel)
       ).withInterrupt(() -> { return this.getShooterMode() != ShooterMode.MANUAL_FIRE;} 
      );

    setDefaultCommand(fullAuto.perpetually());
    
    
    setupConveyorCommands();
    setupShooterCommands();
  }

  private void setupConveyorCommands() {
    // move to seesaw
    frontConveyorFull.whileActiveOnce(
      new StartEndCommand(
        conveyor::start, 
        conveyor::stop, 
        conveyor
      )
    );
  }
  
  private void setupShooterCommands() {
    // set speed
    shooterAutoEnabled.whileActiveOnce(new RunCommand(() -> { flywheel.setFlywheelSpeed(flywheelTable.getRPM(vision.getTargetDistance(vision.getBestTarget()))); }, flywheel));
    shooterAutoEnabled.whileActiveOnce(new RunCommand(() -> turret.setSetpointDegrees(vision.getClosestTarget().getYaw()), turret));
    frontConveyorFull.whileActiveOnce(new RunCommand(() -> {
      feederA.set(0.25);
      feederB.set(0.25);
    }, this));
    vision.setLED(VisionLEDMode.kOff);
    turret.setDefaultCommand(new RunCommand(() -> {
      if (vision.hasTargets()) {
        //turret.setSetpointDegrees(vision.getBestTarget().getYaw());
        turret.setSetpointDegrees(0);
      } else {
        turret.setSetpointDegrees(0);
        //System.out.println("NO TARGET");
      }
    }, turret));
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


  public boolean getShooterActive() {
    return shooterAutoEnabled.get();
  }

  public void setShooterMode(ShooterMode mode) {
    shooterMode = mode;
  }

  public ShooterMode getShooterMode() {
    return shooterMode;
  }

  public boolean getShooterReady() {
    return false;
    //return flywheel.atSetpoint() && turret.atSetpoint();
  }
  

  public enum ShooterMode {
    FULL_AUTO, // turret and flywheel track the goal, ball is fired if present as soon as shooter is ready
    MANUAL_FIRE, // turret and flywheel track the goal, ball is fired on operator command if present
    DUMP, // Turret locks to dead ahead but flywheel is set to minimum
    DISABLED // turret and flywheel do not move, shooting is impossible
  }
  // Methods should be high level actions and command subsystems to achieve the goal
  // TODO: Define what subsytems need, which will inform requirements for this
  
}
