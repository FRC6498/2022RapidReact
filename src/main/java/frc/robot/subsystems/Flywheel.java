// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends SubsystemBase implements Loggable {
  // Hardware
  private final CANSparkMax neo;
  private final RelativeEncoder encoder;
  // Software
  private final BangBangController flywheelBangBang;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private boolean flywheelActive;
  
  public double flywheelSpeedSetpoint;
  @Log
  private double bangBangOutput;
  @Log
  private double feedforwardOutput;
  private double controllerOutput;
  private NetworkTableEntry flywheelSpeedEntry, feedforwardEntry, bangBangEntry;
  
  public Flywheel() {
    neo = new CANSparkMax(rightFlywheelCANId, MotorType.kBrushless);
    encoder = neo.getEncoder();
    flywheelBangBang = new BangBangController(Constants.ShooterConstants.flywheelSetpointToleranceRPM);
    flywheelFeedforward = new SimpleMotorFeedforward(
      flywheelkS, 
      flywheelkV, 
      flywheelkA
    );

    
    neo.restoreFactoryDefaults(true);
    neo.setIdleMode(IdleMode.kCoast);
    neo.setOpenLoopRampRate(flywheelVelocityRampRateSeconds);
    neo.setInverted(true);
    flywheelActive = true;
    flywheelSpeedSetpoint = 1.5;
    NetworkTable teamtable = NetworkTableInstance.getDefault().getTable("team6498");
    flywheelSpeedEntry = teamtable.getEntry("flywheelSpeed");
    feedforwardEntry = teamtable.getEntry("feedforward");
    bangBangEntry = teamtable.getEntry("bangbang");
  }
    
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  @Config
  public void setFlywheelSpeed(double velocity) {
    flywheelSpeedSetpoint = -velocity * 60;
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

  public boolean getFlywheelActive() {
    return false;
    //return flywheelActive;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed() - flywheelSpeedSetpoint) < flywheelSetpointToleranceRPM;
  }
  
  @Override
  public void periodic() {
    flywheelSpeedEntry.setDouble(getFlywheelSpeed());
    flywheelActive = true;
    if (flywheelActive) {
      bangBangOutput = flywheelBangBang.calculate(getFlywheelSpeed(), flywheelSpeedSetpoint);
      feedforwardOutput = flywheelFeedforward.calculate(flywheelSpeedSetpoint);
      controllerOutput = bangBangOutput + 0.9 * feedforwardOutput;
      neo.setVoltage(controllerOutput);
    } else {
      setFlywheelIdle();
    }
  }

}
