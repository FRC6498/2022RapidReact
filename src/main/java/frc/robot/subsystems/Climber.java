// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberMotor;
  Solenoid lock;
  boolean enabled;
  TalonFXConfiguration config;
  double input;
  public Climber() {
    enabled = false;
    config = new TalonFXConfiguration();
    climberMotor = new WPI_TalonFX(ClimberConstants.climberMotorCANId);
    config.closedloopRamp = 0.5;
    config.peakOutputForward = 0.75;
    config.peakOutputReverse = -0.75;
    climberMotor.setInverted(false);
    config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.clearPositionOnLimitF = true;

    lock = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    //setup encoders
    config.slot0.kP = ClimberConstants.climber_kP;
    climberMotor.configAllSettings(config);
    //climberMotor.configForwardSoftLimitThreshold(17860 * 0.5);
    //climberMotor.configForwardSoftLimitThreshold(286300);
    //climberMotor.configReverseSoftLimitThreshold(0);
    //climberMotor.configReverseSoftLimitEnable(false);
    //climberMotor.configForwardSoftLimitEnable(true);
    //climberMotor.setSelectedSensorPosition(0);
    climberMotor.setNeutralMode(NeutralMode.Brake);
    //lock.set(true);
    configStatusFrames();
  }
  @Log
  public double getEncoderPosition() {
    return climberMotor.getSelectedSensorPosition();
  }

  public void setInput(double input) {
    this.input = input;
  }

  public void lockClimber() {
    lock.set(false);

  }

  public void toggleClimber() {
    lock.set(!lock.get());
    enabled = lock.get();
  }

  public void enable() {
    if (this.enabled == true) {
      this.enabled = false;
    } else this.enabled = true;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    climberMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    if (enabled) {
      climberMotor.set(ControlMode.PercentOutput, input);
    } else {
      if (climberMotor.getControlMode() != ControlMode.Disabled) {
        climberMotor.neutralOutput();
      }
    }
    //climberMotor.set(ControlMode.Position, climberMotorSetpoint);
    // This method will be called once per scheduler run
  }

  private void configStatusFrames() {
    climberMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
  }
}