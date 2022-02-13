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
import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase {
  // Hardware
  CANSparkMax flywheelLeft;
  CANSparkMax flywheelRight;
  RelativeEncoder rightEncoder;
  // Software
  BangBangController flywheelBangBang;
  SimpleMotorFeedforward flywheelFeedforward;
  public Flywheel() {
    flywheelLeft = new CANSparkMax(leftFlywheelCANId, MotorType.kBrushless);
    flywheelRight = new CANSparkMax(rightFlywheelCANId, MotorType.kBrushless);
    rightEncoder = flywheelRight.getEncoder();
    flywheelBangBang = new BangBangController(10);
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
    flywheelRight.setOpenLoopRampRate(flywheelVelocityRampRate);
    flywheelLeft.setOpenLoopRampRate(flywheelVelocityRampRate);
  }
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  public void setFlywheelSpeed(double velocity) {
    flywheelRight.set(flywheelBangBang.calculate(rightEncoder.getVelocity(), velocity) * flywheelFeedforward.calculate(velocity));
  }

  public void setFlywheelIdle() {
    flywheelRight.set(0);
  }
}
