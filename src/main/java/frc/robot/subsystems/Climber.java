// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climberMotor;
  Solenoid lock;
  boolean enabled;
  TalonFXConfiguration config;
  double input;
  DutyCycleOut manualControl = new DutyCycleOut(0);

  public Climber() {
    enabled = false;
    config = new TalonFXConfiguration();
    climberMotor = new TalonFX(ClimberConstants.climberMotorCANId);
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
    config.MotorOutput.PeakForwardDutyCycle = 0.75;
    config.MotorOutput.PeakReverseDutyCycle = -0.75;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;

    lock = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    //setup encoders
    config.Slot0.kP = ClimberConstants.climber_kP;
    //climberMotor.configForwardSoftLimitThreshold(17860 * 0.5);
    //climberMotor.configForwardSoftLimitThreshold(286300);
    //climberMotor.configReverseSoftLimitThreshold(0);
    //climberMotor.configReverseSoftLimitEnable(false);
    //climberMotor.configForwardSoftLimitEnable(true);
    //climberMotor.setSelectedSensorPosition(0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(config);
    //lock.set(true);
  }

  public double getEncoderPosition() {
    return climberMotor.getPosition().getValue();
  }

  public void unlockClimber() {
    lock.set(false);
  }

  public Command lockClimber() {
    return runOnce(() -> lock.set(true));
  }

  public Command manualDrive(DoubleSupplier manualInput) {
    return run(() -> {
      if (!lock.get()) {
        climberMotor.setControl(manualControl.withOutput(manualInput.getAsDouble()));
      }
    }).finallyDo(() -> climberMotor.setControl(manualControl.withOutput(0)));
  }

  @Override
  public void periodic() {
    //climberMotor.set(ControlMode.Position, climberMotorSetpoint);
    // This method will be called once per scheduler run
  }
}