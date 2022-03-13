// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase implements Loggable{
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  Solenoid pinPull;
  TalonFXConfiguration config;
  double climberMotorSetpoint;
  public boolean isDown = false;

  boolean predeploy = true;
  double input;
  //private final StallDetector climberStall;
  public Climber() {
    config = new TalonFXConfiguration();
    isDown = true;
    //climberStall = new StallDetector(new PDPSlot(new PDP(), PDPPortNumber.Port8, PDPBreaker.TwentyAmp));
    //climberStall.setStallCurrent(40);
    //climberStall.setMinStallMillis(100);
    climberMotor = new WPI_TalonFX(climberMotorCANId);
    config.closedloopRamp = 1;
    config.peakOutputForward = 0.75;
    config.peakOutputReverse = -0.75;
    climberMotor.setInverted(false);
    config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.clearPositionOnLimitF = true;
    
    //setup encoders
    config.slot0.kP = climber_kP;
    climberMotor.configAllSettings(config);
    //climberMotor.configForwardSoftLimitThreshold(17860 * 0.5);
    //climberMotor.configForwardSoftLimitThreshold(286300);
    //climberMotor.configReverseSoftLimitThreshold(0);
    //climberMotor.configReverseSoftLimitEnable(false);
    //climberMotor.configForwardSoftLimitEnable(true);
    //climberMotor.setSelectedSensorPosition(0);
    climberMotor.setNeutralMode(NeutralMode.Brake);
    configStatusFrames();
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

  @Log.BooleanBox(name = "Climb Complete")
  private boolean getClimbed() {
    return false;//return climberStall.getStallStatus().isStalled == true;
  }

  @Override
  public void periodic() {
    whereClimber(getEncoderPosition());
    //climberStall.updateStallStatus();
    if (getClimbed()) {
      input = 0;

    }
    climberMotor.set(ControlMode.PercentOutput, input);

    //climberMotor.set(ControlMode.Position, climberMotorSetpoint);
    // This method will be called once per scheduler run
  }

  private void configStatusFrames() {
    climberMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
  }
}
