// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase implements Loggable{
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  double climberMotorSetpoint;
  public boolean isDown = false;
  boolean predeploy = true;
  double input;
  public Climber() {
    isDown = true;
    climberMotor = new WPI_TalonFX(climberMotorCANId);
    climberMotor.configClosedloopRamp(1);
    climberMotor.configPeakOutputForward(0.5);
    climberMotor.configPeakOutputReverse(-0.5);
    climberMotor.setInverted(true);
    climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climberMotor.configClearPositionOnLimitF(true, 20);
    climberMotor.configClearPositionOnLimitR(true, 20);
    //setup encoders
    climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberMotor.config_kP(0, climber_kP);
    //climberMotor.configForwardSoftLimitThreshold(17860 * 0.5);
    climberMotor.configForwardSoftLimitThreshold(286300);
    climberMotor.configReverseSoftLimitThreshold(0);
    climberMotor.configReverseSoftLimitEnable(false);
    climberMotor.configForwardSoftLimitEnable(true);
    climberMotor.setSelectedSensorPosition(0);
    climberMotor.setNeutralMode(NeutralMode.Brake);
  }
  @Log
  public double getEncoderPosition() {
    return climberMotor.getSelectedSensorPosition();
  }

  public void setInput(double input) {
    this.input = input;
  }

  // 1 inch = 11660 ticks
  int ticksPerInch = 11660;
  public void lowerClimber()  {
    climberMotorSetpoint = 0;
    isDown = true;
  }

  public void raiseClimber() {
    climberMotorSetpoint = 21 * ticksPerInch;
    isDown = false;
  }
  //@Log
  public double whereClimber(double getEncoderPosition) {
    if (getEncoderPosition < 50) {
      isDown = true;
    } else {
      isDown = false;
    }
    return getEncoderPosition;
  }

  public void toggleClimber() {
    if (isDown) {
      raiseClimber();
    } else {
      lowerClimber();
    }
  }
  @Override
  public void periodic() {
    whereClimber(getEncoderPosition());
    climberMotor.set(ControlMode.PercentOutput, input);

    //climberMotor.set(ControlMode.Position, climberMotorSetpoint);
    // This method will be called once per scheduler run
  }
}
