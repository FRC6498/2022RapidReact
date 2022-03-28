// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.DifferentialDrivePoseEstimator;
import frc.robot.lib.GoalTrack;
import frc.robot.lib.NTHelper;
import io.github.oblarg.oblog.Loggable;
//import frc.robot.lib.PicoColorSensor;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Coordinates all subsystems involving cargo
 * Drivers command Superstructure, which passes it on to the subsystems
 * It makes sure two subsystems are ready for handoff before initiating it
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

  
  //TODO: Create Driver Dashboard
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
  public double frontFeederSpeedRunning = 0.75;
  @Log
  public double rearFeederSpeedRunning = 0.75;
  public double feederSpeedStopped = 0.0; 
  WPI_TalonFX frontFeeder;
  WPI_TalonFX rearFeeder;
  DoubleSolenoid seesaw;
  GoalTrack goalTrack;
  @Log(tabName = "SmartDashboard", name = "Distance to Hub")
  double distance;

  @Log.BooleanBox(name = "Seesaw Position", colorWhenTrue = "yellow", colorWhenFalse = "blue", tabName = "SmartDashboard")
  boolean seesawFront = true;

  ParallelRaceGroup fullAuto;
  ParallelRaceGroup manualShoot;
  ParallelRaceGroup disabled;
  ParallelRaceGroup dump;
  ParallelRaceGroup testing;
  Consumer<ShooterMode> shooterModeUpdater;
  DifferentialDrivePoseEstimator poseEstimator;

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
    frontFeeder = new WPI_TalonFX(10);
    frontFeeder.setInverted(true);
    rearFeeder = new WPI_TalonFX(11);
    frontFeeder.enableVoltageCompensation(true);
    frontFeeder.configVoltageCompSaturation(12);
    rearFeeder.enableVoltageCompensation(true);
    rearFeeder.configVoltageCompSaturation(12);


    seesaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ConveyorConstants.seesawForwardPCMId, Constants.ConveyorConstants.seesawReversePCMId);

    flywheel.setDefaultCommand(new RunCommand(() -> flywheel.setFlywheelSpeed(NTHelper.getDouble("flywheel_speed_target")), flywheel));
    frontConveyor.setDefaultCommand(new RunCommand(() -> frontConveyor.start(), frontConveyor));
    turret.setDefaultCommand(new RunCommand(()-> turret.stop(), turret));
    backConveyor.setDefaultCommand(new RunCommand(backConveyor::start, backConveyor));

    
    shooterReady = new Trigger(this::getShooterReady);
    robotLinedUp = new Trigger(vision::getAligned);
    frontConveyorFull = new Trigger(frontConveyor::isBallPresent);
    backConveyorFull = new Trigger(backConveyor::isBallPresent);
    flywheelEnabled = new Trigger(flywheel::getActive);
    turretEnabled = new Trigger(turret::getActive);
    flyWheelAtSetpoint = new Trigger(()-> {return !flywheel.atSetpoint();});

    poseEstimator = drivetrain.getPoseEstimator();

    setupShooterCommands();
    setShooterMode(ShooterMode.DUMP_HIGH);
  }
  
  private void setupShooterCommands() {
    //frontConveyorFull.whenActive(new InstantCommand(this::seesawToFront));
    //backConveyorFull.and(frontConveyorFull.negate()).whenActive(new InstantCommand(this::seesawToRear));

    // set speed
   

    //frontConveyorFull.whileActiveOnce(new StartEndCommand(this::runFeeder, this::stopFeeder, this));
    //vision.setLED(VisionLEDMode.kOff);
    //turret.setDefaultCommand(new RunCommand(() -> { turret.setSetpointDegrees(0); }, turret));

  }

  public void runFrontFeeder(double percent) {
    if (percent > 0.1) {
      frontFeederSpeedRunning = percent;
    }
    frontFeeder.set(frontFeederSpeedRunning);
    frontFeeder.setNeutralMode(NeutralMode.Coast);
  }

  @Config
  public void runRearFeeder(double percent) {
    if (percent > 0.1) {
      rearFeederSpeedRunning = percent;
    }
    rearFeeder.set(rearFeederSpeedRunning);
    rearFeeder.setNeutralMode(NeutralMode.Coast);
  }

  @Config
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

  public void seesawToRear() {
    seesaw.set(Value.kForward);
  }

  public void seesawToFront() {
    seesaw.set(Value.kReverse);
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
      case DUMP_HIGH:
      case DUMP_LOW:
        NetworkTableInstance.getDefault().setUpdateRate(0.1);
        vision.setLED(VisionLEDMode.kOff);
        vision.setDriverMode(true);
        break;
      default:
        break;
    }
  }

  public void toggleSeesaw() {
    if (seesawFront) {
      seesawToRear();
    } else {
      seesawToFront();
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
    DUMP_LOW, // Turret locks to dead ahead but flywheel is set to minimum
    DUMP_HIGH, // Turret locks to dead ahead but flywheel is set to high goal speed
    HOMING, //
    TUNING, // Turret Disabled, flywheel speed settable
    DISABLED // turret and flywheel do not move, shooting is impossible
  }
  // setShooterMode method here
  // subsystems check shooter mode, act accordingly

  public void addVisionUpdate(double timestamp, PhotonTrackedTarget target) {
    //counter clock wise is positivie thats why target yaw is inverted
    //pretty sure x is positive forward, y is positive left
        
    double yaw = -target.getYaw();
  
    distance = vision.getTargetDistance(target);
    Rotation2d angle = turret.getCurrentPosition().plus(Rotation2d.fromDegrees(yaw)).plus(drivetrain.getRotation2d());
    Translation2d field_to_goal=new Translation2d(distance * angle.getCos(), distance * angle.getSin());
    goalTrack.tryUpdate(timestamp, field_to_goal);
    // System.out.println("time: "+timestamp+ " x: "+field_to_goal.getX()+" y: "+field_to_goal.getY());
    

}

  public void updateVision() {
    if (vision.hasTargets()) {
      poseEstimator.addVisionMeasurement(
        PhotonUtils.estimateFieldToRobot(
          VisionConstants.limelightHeightFromField, 
          VisionConstants.upperHubTargetHeight, 
          VisionConstants.limelightPitch, 
          vision.getBestTarget().getPitch(), 
          Rotation2d.fromDegrees(vision.getBestTarget().getYaw()), 
          drivetrain.getRotation2d(), 
          VisionConstants.fieldToTargetTransform, 
          new Transform2d(
            new Translation2d(
              Units.inchesToMeters(6), 
              Units.inchesToMeters(-(VisionConstants.limelightHeightFromField-2))
            ), 
          turret.getCurrentPosition())
        ), 
        Timer.getFPGATimestamp()
      );
      addVisionUpdate(Timer.getFPGATimestamp(), vision.getBestTarget());
    }
  }

  public boolean getSeesawFront() {
    return seesaw.get() == Value.kReverse;
  }

  @Override
  public void periodic() {
    
    Translation2d smoothedPos = goalTrack.getSmoothedPosition();
    // ALWAYS MAKE A NEW VARIABLE, OTHERWISE IT WILL JITTER DANGEROUSLY!!!
    Rotation2d smoothedRotation = new Rotation2d(smoothedPos.getX(), smoothedPos.getY());
    NTHelper.setDouble("smoothed_pos_angle", smoothedRotation.getDegrees());
    //NTHelper.setDouble("smooth_pos_deg", value);
    switch (mode) {
      case MANUAL_FIRE:
        if (goalTrack.hasData()) {
          //turret.setPositionSetpoint(smoothedRotation.minus(drivetrain.getGyroAngle()));
        }
        break;
      default:
        break;
    }
    if (turret.getCurrentPosition().getDegrees() > -10) { // 0, front
      turretAtFront = true;
    } else {
      turretAtFront = false;
    }
    
    if (frontIntake.isExtended()) {
      seesawToFront();
    } else if (backIntake.isExtended()) {
      seesawToRear();
    }
  }
  
}
