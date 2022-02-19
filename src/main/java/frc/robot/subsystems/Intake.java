// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase implements Loggable {
  WPI_TalonFX frontMotor;
  
  WPI_TalonFX backMotor;
  Double frontMotorSetpoint;
  Double backMotorSetpoint;
  DoubleSolenoid frontIntake;
  DoubleSolenoid backIntake;
  /** Creates a new Intake. */
  public void frontIntake() {
    frontMotor = new WPI_TalonFX (intakeACANId);
    frontMotor.configOpenloopRamp(1);
    frontMotorSetpoint = 0.0;
    frontIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, frontIntakeForwardChannel, frontIntakeReverseChannel);
    
  }
  public void backIntake() {
    backMotor = new WPI_TalonFX (intakeBCANId);
    backMotor.configOpenloopRamp(1);
    backMotorSetpoint = 0.0;
    backIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, backIntakeForwardChannel, backIntakeReverseChannel);

  }

  public void lowerFrontIntake() {
    frontMotorSetpoint = 0.67;
    frontIntake.set(Value.kReverse);
  }

  public void raiseFrontIntake() {
    frontMotorSetpoint = 0.0;
    frontIntake.set(Value.kForward);
  }

  public void lowerBackIntake() {
    backMotorSetpoint = 0.67;
    backIntake.set(Value.kReverse);
  }

  public void raiseBackIntake() {
    backMotorSetpoint = 0.0; 
    backIntake.set(Value.kForward);
  }


  @Config
  public void setfrontMotorSetpoint(double percent) {
    frontMotorSetpoint = percent;
  }
  public void setbackMotorSetpoint(double percent) {
    backMotorSetpoint = percent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
