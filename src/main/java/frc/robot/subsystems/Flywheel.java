// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.RotationsPerMinute;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.InterpolatingTable;

public class Flywheel extends SubsystemBase implements Logged {
  // Hardware
  private final TalonFX shooter;
  private final TalonFX hoodRollers;
  // Software
  public MutableMeasure<Velocity<Angle>> flywheelSpeedSetpoint = RotationsPerMinute.of(-3000.0).mutableCopy();
  private MutableMeasure<Velocity<Angle>> hoodTargetRPM = RotationsPerSecond.of(0).mutableCopy();
  private TalonFXConfiguration hoodConfig;
  private VelocityVoltage velocityMode = new VelocityVoltage(0);

  //private double feedforwardOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  //@Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  
  public Flywheel() {
    shooter = new TalonFX(ShooterConstants.flywheelCANId);
    hoodRollers = new TalonFX(ShooterConstants.hoodRollerCANId);
    hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = 0.046642;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
    hoodConfig.Slot0.kS = 0.55;
    hoodConfig.Slot0.kV = 12.3 / 6380.0;
    hoodConfig.Slot0.kA = 0;
    // once we hit 40A for >=100ms, hold at 40A
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodRollers.getConfigurator().apply(hoodConfig);
  }
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  //@Config(name = "Set Flywheel Speed(RPM)")
  public void setFlywheelSpeed(Measure<Velocity<Angle>> velocity) {
    flywheelSpeedSetpoint = velocity.negate().mutableCopy();
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  public Measure<Velocity<Angle>> getHoodSpeed() {
    return RotationsPerSecond.of(hoodRollers.getVelocity().getValue());
  }

  //@Log(name = "Flywheel Velocity (RPM)")
  public Measure<Velocity<Angle>> getFlywheelSpeed() {
    return RotationsPerSecond.of(shooter.getVelocity().getValue());
  }

  @Log
  public Measure<Velocity<Angle>> getHoodError() {
    return getHoodSpeed().minus(hoodTargetRPM);
  }

  public void setFlywheelIdle() {
    shooter.setControl(velocityMode.withVelocity(0));
  }

  public boolean getActive() {
    return shooter.getClosedLoopReference().getValue() > 1;
  }

  public boolean atSetpoint() {
    return shooter.getClosedLoopError().getValue() * 60 < ShooterConstants.flywheelSetpointToleranceRPM;
  }

  public Command manualFire() {
    return run(() -> {
      flywheelSpeedSetpoint.mut_replace(InterpolatingTable.get(distanceToHub).shooterSpeed);
      hoodTargetRPM.mut_replace(flywheelSpeedSetpoint.plus(InterpolatingTable.get(distanceToHub).hoodSpeedOffset));
      shooter.setControl(velocityMode.withVelocity(flywheelSpeedSetpoint.in(RotationsPerSecond)));
      hoodRollers.setControl(velocityMode.withVelocity(hoodTargetRPM.in(RotationsPerSecond)));
    });
  }

  public Command reject() {
    var rejectShooterSpeed = RotationsPerSecond.of(1000).mutableCopy();
    var rejectHoodSpeed = rejectShooterSpeed;
    flywheelSpeedSetpoint = rejectShooterSpeed;
    hoodTargetRPM = rejectHoodSpeed;
    return runOnce(() -> {
      shooter.setControl(velocityMode.withVelocity(rejectShooterSpeed.in(RotationsPerSecond)));
      hoodRollers.setControl(velocityMode.withVelocity(rejectHoodSpeed.in(RotationsPerSecond)));
    }).andThen(Commands.idle(this));
  }

  public Command idle() {
    flywheelSpeedSetpoint = RotationsPerSecond.zero().mutableCopy();;
    hoodTargetRPM = RotationsPerSecond.zero().mutableCopy();;
    return runOnce(() -> {
      shooter.setControl(velocityMode.withVelocity(0));
      hoodRollers.setControl(velocityMode.withVelocity(0));
    }).andThen(Commands.idle(this));
  }

  @Override
  public void periodic() {}

}
