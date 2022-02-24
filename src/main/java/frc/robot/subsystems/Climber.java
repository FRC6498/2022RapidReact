// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  public Climber() {
    climberMotor = new WPI_TalonFX(climberMotorCANId);
    climberMotor.configOpenloopRamp(1);
  
  }

  public void toggle() {

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
