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
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  double climberMotorSetpoint;
  public boolean isDown = false;
  boolean predeploy = true;
  public Climber() {
    isDown = true;
    climberMotor = new WPI_TalonFX(climberMotorCANId);
    climberMotor.configClosedloopRamp(1);
    //setup encoders
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberMotor.setSelectedSensorPosition(357205);
  }
  public double getEncoderPosition() {
    return climberMotor.getSelectedSensorPosition();
  }

  public void lowerClimber()  {
    climberMotorSetpoint = 0;
    climberMotor.setNeutralMode(NeutralMode.Brake);
    climberMotor.set(ControlMode.Position, climberMotorSetpoint);
  }

  public void releaseClimber() {
    isDown = false;
    climberMotor.setNeutralMode(NeutralMode.Coast);
    climberMotor.set(ControlMode.PercentOutput, 0);
  }
  @Log
  public double whereClimber(double getEncoderPosition) {
    if (getEncoderPosition < 50) {
      isDown = true;
    } else {
      isDown = false;
    }
    return getEncoderPosition;
  }
  public void toggleClimber() {
    if (isDown && predeploy) { // raise climber
      releaseClimber();
      predeploy = false;
    } else if (!isDown && !predeploy) { // retract
      lowerClimber();
      predeploy = true;
    }
  }
  @Override
  public void periodic() {
    whereClimber(getEncoderPosition());
    // This method will be called once per scheduler run
  }
}
