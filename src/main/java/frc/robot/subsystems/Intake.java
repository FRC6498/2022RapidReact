// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Intake extends SubsystemBase implements Loggable {
  WPI_TalonFX motor;
  Double motorSetpoint;
  DoubleSolenoid piston;

  public Intake(int intakeMotorId, int pistonForwardId, int pistonReverseId) {
    motor = new WPI_TalonFX(intakeMotorId);
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pistonForwardId, pistonReverseId);
    piston.set(Value.kForward);
  }

  public void lowerIntake() {
    setMotorSetpoint(0.67);
    piston.set(Value.kReverse);
  }

  public void raiseIntake() {
    setMotorSetpoint(0.0);
    piston.set(Value.kForward);
  }

  //@Config
  public void setMotorSetpoint(double percent) {
    motorSetpoint = percent;
  }

  public boolean isExtended() {
    return motorSetpoint > 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
