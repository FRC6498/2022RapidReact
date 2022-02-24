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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
  private double flywheelSpeedSetpoint;
  private double bangBangOutput;
  private double feedforwardOutput;
  private double controllerOutput;
  private double lastLoopPosition;
  public DoubleSolenoid tickTock;
  
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

    flywheelActive = false;
    flywheelSpeedSetpoint = 0.0;
    }
    
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  public void setFlywheelSpeed(double velocity) {
    flywheelSpeedSetpoint = velocity;
  }

  @Log(name = "Flywheel Velocity (RPM)")
  public double getFlywheelSpeed() {
    return encoder.getVelocity();
  }

  public void setFlywheelIdle() {
    neo.set(0);
  }

  @Config.ToggleButton
  public void setFlywheelActive(boolean active) {
    flywheelActive = active;
  }

  public boolean getFlywheelActive() {
    return flywheelActive;
  }

  public boolean atSetpoint() {
    return Math.abs(encoder.getVelocity() - flywheelSpeedSetpoint) < flywheelSetpointToleranceRPM;
  }

  @Override
  public void periodic() {

    if (flywheelActive) {
      bangBangOutput = flywheelBangBang.calculate((encoder.getPosition() - lastLoopPosition) / 0.02, flywheelSpeedSetpoint);
      feedforwardOutput = flywheelFeedforward.calculate(flywheelSpeedSetpoint);
      controllerOutput = bangBangOutput + 0.9 * feedforwardOutput;
      neo.set(controllerOutput);
    } else {
      setFlywheelIdle();
    }
    lastLoopPosition = encoder.getPosition();
  }

}
