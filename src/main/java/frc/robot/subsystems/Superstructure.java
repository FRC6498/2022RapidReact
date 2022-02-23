// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.PicoColorSensor;
import frc.robot.lib.ShotMap;
import io.github.oblarg.oblog.annotations.Config;

/**
 * Coordinates all subsystems involving cargo
 * Drivers command Superstructure, which passes it on to the subsystems
 * It makes sure two subsystems are ready for handoff before initiating it
 * Reports Statuses back to the dashboard
 */
public class Superstructure extends SubsystemBase {
  // Intake
  private final Intake frontIntake, backIntake;
  // Vision
  private final Vision vision;
  // Conveyor
  private final Conveyor frontConveyor, backConveyor;
  
  //private final PicoColorSensor colorSensor;
  // Flywheel
  private final Flywheel flywheel;
  // Turret
  private final Turret turret;
  // Feeder

  // Triggers
  // Superstructure
  public Trigger shooterReady;
  public Trigger seesawReady;
  // Conveyors
  public Trigger frontConveyorFull;
  public Trigger frontConveyorBallColorCorrect;
  public Trigger backConveyorFull;
  public Trigger backConveyorBallColorCorrect;
  // Intakes
  // Flywheel
  public Trigger shooterAutoEnabled;
  ShotMap flywheelTable = new ShotMap();
  ShooterMode shooterMode;
  @Config
  double flywheelRPM = 0.0;

  public Superstructure(Flywheel flywheel, Conveyor frontConveyor, Conveyor backConveyor, Intake frontIntake, Intake backIntake, Vision vision, Turret turret) {
    this.flywheel = flywheel;
    this.frontConveyor = frontConveyor;
    this.backConveyor = backConveyor;
    this.frontIntake = frontIntake;
    this.backIntake = backIntake;
    this.turret = turret;
    this.vision = vision;
    //colorSensor = new PicoColorSensor();



    flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setFlywheelIdle(), flywheel));
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.stop(), frontConveyor));
    backConveyor.setDefaultCommand(new RunCommand(() -> backConveyor.stop(), backConveyor));
    turret.setDefaultCommand(new RunCommand(()-> turret.stop(), turret));

    shooterReady = new Trigger(this::getShooterReady);
    seesawReady = new Trigger();
    frontConveyorFull = new Trigger(() -> frontConveyor.isBallPresent(false));
    frontConveyorBallColorCorrect = new Trigger(() -> {return frontConveyor.getCargoColor() == this.getAllianceColor(); });
    backConveyorFull = new Trigger(() -> backConveyor.isBallPresent(false));
    backConveyorBallColorCorrect = new Trigger(() -> {return backConveyor.getCargoColor() == this.getAllianceColor(); });
    shooterAutoEnabled = new Trigger(flywheel::getFlywheelActive);

    this.shooterMode = ShooterMode.FULL_AUTO;
    setupConveyorCommands();
    setupShooterCommands();
  }

  

  private void setupConveyorCommands() {
    // move to seesaw
    //shooterReady.and(frontConveyorFull).and(frontConveyorBallColorCorrect).and(seesawReady).whileActiveOnce(
    //  new StartEndCommand(
    //    frontConveyor::start, 
    //    frontConveyor::stop, 
    //    frontConveyor
    //  )
    //);
    //shooterReady.and(backConveyorFull).and(backConveyorBallColorCorrect).and(seesawReady).whileActiveOnce(
    //  new StartEndCommand(
    //    backConveyor::start, 
    //    backConveyor::stop, 
    //    backConveyor
    //  )
    //);
    
  }

  private void setupShooterCommands() {
    // set speed
    shooterAutoEnabled.whileActiveOnce(new RunCommand(() -> { flywheel.setFlywheelSpeed(flywheelTable.getRPM(vision.getTargetDistance(vision.getBestTarget()))); }, flywheel));
    shooterAutoEnabled.whileActiveOnce(new RunCommand(() -> turret.setSetpointDegrees(vision.getClosestTarget().getYaw()), turret));
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

  public void setShooterActive(ShooterMode mode) {
    shooterMode = mode;
  }

  public boolean getShooterReady() {
    return flywheel.atSetpoint() && turret.atSetpoint();
  }

  public enum ShooterMode {
    FULL_AUTO, // turret and flywheel track the goal, ball is fired if present as soon as shooter is ready
    MANUAL_FIRE, // turret and flywheel track the goal, ball is fired on operator command if present
    TRACK_ONLY, // Turret tracks the goal but flywheel coats down
    DISABLED // turret and flywheel do not move, shooting is impossible
  }
  // Methods should be high level actions and command subsystems to achieve the goal
  // TODO: Define what subsytems need, which will inform requirements for this

}
