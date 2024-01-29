// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.RotationsPerMinute;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterMode;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.InterpolatingTable;

public class Flywheel extends SubsystemBase implements Logged {
  // Hardware
  private final TalonFX shooter;
  private final TalonFX hoodRollers;
  // Software
  private final SimpleMotorFeedforward hoodFeedforward;
  private boolean flywheelActive;
  public Measure<Velocity<Angle>> flywheelSpeedSetpoint = RotationsPerMinute.of(-3000.0);
  private Measure<Velocity<Angle>> speedOffset = RotationsPerMinute.of(0.0);
  private Measure<Velocity<Angle>> hoodTargetRPM = RotationsPerSecond.of(0);
  private TalonFXConfiguration hoodConfig;
  private VelocityVoltage velocityMode = new VelocityVoltage(0);

  //private double feedforwardOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  //@Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  private ShooterMode mode;
  
  public Flywheel() {
    mode = ShooterMode.DISABLED;
    shooter = new TalonFX(ShooterConstants.flywheelCANId);
    hoodFeedforward = new SimpleMotorFeedforward(
      0.55,
      12.3 / 6380.0, 
      0
    );
    hoodRollers = new TalonFX(ShooterConstants.hoodRollerCANId);
    hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kP = 0.046642;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
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
    flywheelSpeedSetpoint = velocity.negate();
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  //@Config
  public void setHoodRollerOffset(Measure<Velocity<Angle>> rpmOffset) {
    speedOffset = rpmOffset.copy();
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
    return mode != ShooterMode.HOMING;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed().minus(flywheelSpeedSetpoint).in(RotationsPerMinute)) < ShooterConstants.flywheelSetpointToleranceRPM;
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
    switch (mode) {
      case MANUAL_FIRE:
      case REJECT:
        flywheelActive = true;
        break;
      case DISABLED:
      case HOMING:
        flywheelActive = false;
        break;
      default:
      flywheelActive = false;
        break;
    }
  }

  @Override
  public void periodic() {
    switch (mode) {
      case MANUAL_FIRE:
        setFlywheelSpeed(InterpolatingTable.get(distanceToHub).shooterSpeed);
        setHoodRollerOffset(InterpolatingTable.get(distanceToHub).hoodSpeedOffset);
        break;
      case REJECT:
        setFlywheelSpeed(RotationsPerMinute.of(1000));
        setHoodRollerOffset(RotationsPerMinute.of(0));
      default:
        break;
    }
      
    /*} else if (mode == ShooterMode.DUMP_LOW) {
      setFlywheelSpeed(1750); // dump = 1750
    } else if (mode == ShooterMode.DUMP_HIGH) {
      setFlywheelSpeed(3500);*/
    //flywheelSpeedSetpoint = MathUtil.clamp(flywheelSpeedSetpoint, -6500, -1000);
    if (flywheelActive) {
      hoodTargetRPM = flywheelSpeedSetpoint.plus(speedOffset);
      double feedforwardOutput = hoodFeedforward.calculate(hoodTargetRPM.in(RotationsPerMinute)) * 0.97;
      //pid.setReference(flywheelSpeedSetpoint, ControlType.kVelocity);
      hoodRollers.setControl(velocityMode.withVelocity(hoodTargetRPM.in(RotationsPerSecond)).withFeedForward(feedforwardOutput));
    } else {
      setFlywheelIdle();
    }
  }

}
