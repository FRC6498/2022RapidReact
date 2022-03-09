// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.ShotMap;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends SubsystemBase implements Loggable {
  // Hardware
  private final CANSparkMax neo;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  // Software
  private final BangBangController flywheelBangBang;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(
    flywheelkV, 
    flywheelkA
  );
  private final KalmanFilter<N1, N1, N1> observer =new KalmanFilter<>(
    Nat.N1(), 
    Nat.N1(), 
    flywheelPlant, 
    VecBuilder.fill(3.0),
    VecBuilder.fill(0.01), 
    0.020
  );
  private boolean flywheelActive;
  
  public double flywheelSpeedSetpoint;
  @Log
  private double bangBangOutput;
  @Log
  private double feedforwardOutput;
  @Log 
  private double pidOutput;
  @Log
  private double controllerOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  private NetworkTableEntry flywheelSparkMAXSpeedEntry, flywheelSpeedPositionDifferenceEntry;
  @Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  private ShooterMode mode;
  ShotMap flywheelTable = new ShotMap();
  
  public Flywheel() {
    mode = ShooterMode.DISABLED;
    flywheelTable.put(0, 0.8);
    flywheelTable.put(Units.feetToMeters(9), 1.6);
    flywheelTable.put(Units.inchesToMeters(10*12+4), 1.8);
    neo = new CANSparkMax(rightFlywheelCANId, MotorType.kBrushless);
    encoder = neo.getEncoder();
    flywheelBangBang = new BangBangController(Constants.ShooterConstants.flywheelSetpointToleranceRPM);
    flywheelFeedforward = new SimpleMotorFeedforward(
      flywheelkS, 
      flywheelkV, 
      flywheelkA
    );
    pid = neo.getPIDController();
    
    neo.restoreFactoryDefaults(true);
    neo.setIdleMode(IdleMode.kCoast);
    neo.setOpenLoopRampRate(flywheelVelocityRampRateSeconds);
    neo.setInverted(true);
    neo.enableVoltageCompensation(12);
    pid.setP(0.12528);
    pid.setI(0);
    pid.setD(0);
    pid.setFF(0);
    flywheelActive = true;
    flywheelSpeedSetpoint = 1.5;
    NetworkTable teamtable = NetworkTableInstance.getDefault().getTable("team6498");
    flywheelSparkMAXSpeedEntry = teamtable.getEntry("flywheelSpeedEncoderVelocity");
    flywheelSpeedPositionDifferenceEntry = teamtable.getEntry("flywheelSpeedPositionDifference");
  } 
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  @Config
  public void setFlywheelSpeed(double velocity) {
    flywheelSpeedSetpoint = -velocity * 60;
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  @Log(name = "Flywheel Velocity (RPM)")
  public double getFlywheelSpeed() {
    return encoder.getVelocity();
  }
  // input -> rpm conversion rate is approx. 1 : 3000

  public void setFlywheelIdle() {
    neo.set(0.0);
  }

  @Config.ToggleButton
  public void setFlywheelActive(boolean active) {
    flywheelActive = active;
  }

  public boolean getActive() {
    return flywheelActive;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed() - flywheelSpeedSetpoint) < flywheelSetpointToleranceRPM;
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
  }
  
  @Override
  public void periodic() {
    flywheelSparkMAXSpeedEntry.setDouble(getFlywheelSpeed());
    flywheelSpeedPositionDifferenceEntry.setDouble((encoder.getPosition() - lastPosition) / 0.02);
    // set setpoint based on mode
    switch (mode) {
      case DISABLED:
        flywheelSpeedSetpoint = 0;
        flywheelActive = true;
        break;
      case DUMP:
        flywheelSpeedSetpoint = flywheelTable.getRPM(0);
        flywheelActive = true;
      break;
      case FULL_AUTO:
      flywheelSpeedSetpoint = flywheelTable.getRPM(distanceToHub);
      default:
        break;
    }
    flywheelActive = true;
    if (flywheelActive) {
      /*bangBangOutput = flywheelBangBang.calculate(getFlywheelSpeed(), flywheelSpeedSetpoint);
      feedforwardOutput = flywheelFeedforward.calculate(flywheelSpeedSetpoint);
      controllerOutput = bangBangOutput + 0.9 * feedforwardOutput;
      neo.setVoltage(controllerOutput);*/
      pid.setReference(flywheelSpeedSetpoint, ControlType.kVelocity, 0, flywheelFeedforward.calculate(flywheelSpeedSetpoint), ArbFFUnits.kVoltage);
    } else {
      setFlywheelIdle();
    }

    lastPosition = encoder.getPosition();
  }

}
