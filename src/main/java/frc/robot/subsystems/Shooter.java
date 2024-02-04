// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.RotationsPerMinute;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.InterpolatingTable;
import frc.robot.simulation.ShooterSim;

public class Shooter extends SubsystemBase implements Logged {
  // Hardware
  private final TalonFX shooter;
  private final TalonFX hoodRollers;
  // Software
  public final MutableMeasure<Velocity<Angle>> flywheelSpeedSetpoint = RotationsPerMinute.of(-3000.0).mutableCopy();
  private final MutableMeasure<Velocity<Angle>> hoodTargetRPM = RotationsPerSecond.of(0).mutableCopy();
  private final VelocityVoltage velocityMode = new VelocityVoltage(0);
  private final ShooterSim sim;
  private final DoubleSubscriber manualSpeedData;

  public Shooter() {
    shooter = new TalonFX(ShooterConstants.flywheelCANId);
    hoodRollers = new TalonFX(ShooterConstants.hoodRollerCANId);
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = 0.046642;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
    hoodConfig.Slot0.kS = 0.55;
    hoodConfig.Slot0.kV = 12.3 / 6380.0;
    hoodConfig.Slot0.kA = 0;
    // once we hit 40A for >=100ms, hold at 40A
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodRollers.getConfigurator().apply(hoodConfig);

    flywheelConfig.Slot0.kS = ShooterConstants.flywheelkS;
    flywheelConfig.Slot0.kV = ShooterConstants.flywheelkV;
    flywheelConfig.Slot0.kA = ShooterConstants.flywheelkA;
    flywheelConfig.Slot0.kP = ShooterConstants.flywheelkP;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooter.getConfigurator().apply(flywheelConfig);

    sim = new ShooterSim(shooter);
    manualSpeedData = NetworkTableInstance.getDefault().getDoubleTopic("Robot/flywheelSpeedRpm").subscribe(0, PubSubOption.keepDuplicates(true));
  }
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  //@Config(name = "Set Flywheel Speed(RPM)")
  public void setFlywheelSpeed(Measure<Velocity<Angle>> velocity) {
    flywheelSpeedSetpoint.mut_replace(velocity.negate());
  }

  public Measure<Velocity<Angle>> getHoodSpeed() {
    return RotationsPerSecond.of(hoodRollers.getVelocity().getValue());
  }

  @Log //(name = "Flywheel Velocity (RPM)")
  public Measure<Velocity<Angle>> getFlywheelSpeed() {
    log("flywheelRPM", shooter.getVelocity().getValue() * 60.0);
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

  public Command manualFire(Supplier<OptionalDouble> distance) {
    return run(() -> {
      distance.get().ifPresent((hubDistance) -> {
        flywheelSpeedSetpoint.mut_replace(InterpolatingTable.get(hubDistance).shooterSpeed);
      hoodTargetRPM.mut_replace(flywheelSpeedSetpoint.plus(InterpolatingTable.get(hubDistance).hoodSpeedOffset));
      });
      shooter.setControl(velocityMode.withVelocity(flywheelSpeedSetpoint.in(RotationsPerSecond)));
      hoodRollers.setControl(velocityMode.withVelocity(hoodTargetRPM.in(RotationsPerSecond)));
    });
  }

  public Command manualSpeed() {
    log("Heartbeat", "Shooter!");
    return run(() -> {
      flywheelSpeedSetpoint.mut_replace(manualSpeedData.get(0), RotationsPerMinute);
      shooter.setControl(velocityMode.withVelocity(flywheelSpeedSetpoint.in(RotationsPerSecond)));
      hoodRollers.setControl(velocityMode.withVelocity(hoodTargetRPM.in(RotationsPerSecond)));
    });
  }

  public Command reject() {
    var rejectShooterSpeed = RotationsPerSecond.of(1000).mutableCopy();
    var rejectHoodSpeed = rejectShooterSpeed;
    flywheelSpeedSetpoint.mut_replace(rejectShooterSpeed);
    hoodTargetRPM.mut_replace(rejectHoodSpeed);
    return runOnce(() -> {
      shooter.setControl(velocityMode.withVelocity(rejectShooterSpeed.in(RotationsPerSecond)));
      hoodRollers.setControl(velocityMode.withVelocity(rejectHoodSpeed.in(RotationsPerSecond)));
    }).andThen(Commands.idle(this));
  }

  public Command idle() {
    flywheelSpeedSetpoint.mut_replace(RotationsPerSecond.zero());
    hoodTargetRPM.mut_replace(RotationsPerSecond.zero());
    return runOnce(() -> {
      shooter.setControl(velocityMode.withVelocity(0));
      hoodRollers.setControl(velocityMode.withVelocity(0));
    }).andThen(Commands.idle(this));
  }

  @Override
  public void simulationPeriodic() {
    sim.updateSim();
  }

}
