// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  double climberMotorSetpoint;
  public boolean isDown;
  public Climber() {
    climberMotor = new WPI_TalonFX(climberMotorCANId);
    climberMotor.configOpenloopRamp(1);
    climberMotor.set(ControlMode.Position, climberMotorSetpoint);
    //setup encoders
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }
public double getEncoderPosition() {
  return climberMotor.getSelectedSensorPosition();
}
  public void lowerClimber()  {
    climberMotorSetpoint = 0;
  }
  public void releaseClimber() {
    climberMotor.setNeutralMode(NeutralMode.Coast);
  }
  
  public void whereClimber(double getEncoderPosition) {
    if (getEncoderPosition < 50) {
      isDown = true;
      
    } else {
      isDown = false;
    }
  }
  public void toggleClimber() {
    if (isDown) {
      climberMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      climberMotor.setNeutralMode(NeutralMode.Coast);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
