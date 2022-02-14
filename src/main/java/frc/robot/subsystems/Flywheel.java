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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends SubsystemBase implements Loggable {
  // Hardware
  CANSparkMax flywheelLeft;
  CANSparkMax flywheelRight;
  RelativeEncoder rightEncoder;
  // Software
  BangBangController flywheelBangBang;
  SimpleMotorFeedforward flywheelFeedforward;
  boolean flywheelActive;
  double flywheelSpeedSetpoint;
  double bangBangOutput;
  double feedforwardOutput;
  double controllerOutput;
  public Flywheel() {
    flywheelLeft = new CANSparkMax(leftFlywheelCANId, MotorType.kBrushless);
    flywheelRight = new CANSparkMax(rightFlywheelCANId, MotorType.kBrushless);
    rightEncoder = flywheelRight.getEncoder();
    flywheelBangBang = new BangBangController(Constants.ShooterConstants.flywheelSetpointToleranceRPM);
    flywheelFeedforward = new SimpleMotorFeedforward(
      flywheelkS, 
      flywheelkV, 
      flywheelkA
    );

    flywheelRight.restoreFactoryDefaults(true);
    flywheelLeft.restoreFactoryDefaults(true);
    // invert follower because it is pointing the opposite direction
    flywheelLeft.follow(flywheelRight, true);
    // coast flywheel motors so BangBang doesnt go wild
    flywheelLeft.setIdleMode(IdleMode.kCoast);
    flywheelRight.setIdleMode(IdleMode.kCoast);
    flywheelRight.setOpenLoopRampRate(flywheelVelocityRampRateSeconds);
    flywheelLeft.setOpenLoopRampRate(flywheelVelocityRampRateSeconds);

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
    return rightEncoder.getVelocity();
  }

  public void setFlywheelIdle() {
    flywheelRight.set(0);
  }

  @Config.ToggleButton
  public void setFlywheelActive(boolean active) {
    flywheelActive = active;
  }

  @Override
  public void periodic() {
    if (flywheelActive) {
      bangBangOutput = flywheelBangBang.calculate(rightEncoder.getVelocity(), flywheelSpeedSetpoint);
      feedforwardOutput = flywheelFeedforward.calculate(flywheelSpeedSetpoint);
      controllerOutput = bangBangOutput + 0.9 * feedforwardOutput;
      flywheelRight.set(controllerOutput);
    } else {
      setFlywheelIdle();
    }
  }
}
