// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.InterpolatingTable;

public class Flywheel extends SubsystemBase implements Loggable {
  // Hardware
  private final CANSparkMax neo;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  private final WPI_TalonFX hoodRollers;
  // Software
  private final SimpleMotorFeedforward hoodFeedforward;
  private boolean flywheelActive;
  @Log
  public double flywheelSpeedSetpoint = -3000.0;
  @Log
  private double speedOffset = 0.0;
  @Log
  private double hoodTargetRPM = 0.0;
  private TalonFXConfiguration hoodConfig;
  private final double nominalVoltage = 12.3;

  //private double feedforwardOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  //@Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  private ShooterMode mode;
  public Flywheel() {
    mode = ShooterMode.DISABLED;
    neo = new CANSparkMax(ShooterConstants.flywheelCANId, MotorType.kBrushless);
    encoder = neo.getEncoder();
    hoodFeedforward = new SimpleMotorFeedforward(
      0.55,
      nominalVoltage / 6380.0, 
      0
    );
    pid = neo.getPIDController();
    hoodRollers = new WPI_TalonFX(ShooterConstants.hoodRollerCANId);
    hoodConfig = new TalonFXConfiguration();
    hoodConfig.slot0.kP = 0.046642;
    hoodConfig.slot0.kI = 0;
    hoodConfig.slot0.kD = 0;
    hoodConfig.slot0.kF = 0;
    // once we hit 40A for >=100ms, hold at 40A
    hoodConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 100);
    hoodRollers.configAllSettings(hoodConfig);
    hoodRollers.setInverted(TalonFXInvertType.Clockwise);
    neo.enableVoltageCompensation(nominalVoltage);
    hoodRollers.configVoltageCompSaturation(nominalVoltage);
    hoodRollers.enableVoltageCompensation(true);
  }
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  //@Config(name = "Set Flywheel Speed(RPM)")
  @Config
  public void setFlywheelSpeed(double velocity) {
    flywheelSpeedSetpoint = -velocity;
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  @Config
  public void setHoodRollerOffset(double rpmOffset) {
    speedOffset = rpmOffset;
  }

  @Log
  public double getHoodSpeed() {
    return falconTicksToRPM(hoodRollers.getSelectedSensorVelocity());
  }

  //@Log(name = "Flywheel Velocity (RPM)")
  @Log
  public double getFlywheelSpeed() {
    return encoder.getVelocity();
  }

  @Log
  public double getHoodError() {
    return getHoodSpeed() - hoodTargetRPM;
  }

  public void setFlywheelIdle() {
    pid.setReference(0, ControlType.kDutyCycle);
  }

  public boolean getActive() {
    return mode != ShooterMode.HOMING;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed() - flywheelSpeedSetpoint) < ShooterConstants.flywheelSetpointToleranceRPM;
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

  private double rpmToNativeUnits(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  private double falconTicksToRPM(double ticks) {
    return ticks / 2048.0 * 600.0;
  }

  @Override
  public void periodic() {
    switch (mode) {
      case MANUAL_FIRE:
        setFlywheelSpeed(InterpolatingTable.get(distanceToHub).rpm);
        setHoodRollerOffset(InterpolatingTable.get(distanceToHub).hoodSpeedOffset);
        break;
      case REJECT:
        setFlywheelSpeed(1000);
        setHoodRollerOffset(0);
      default:
        break;
    }
      
    /*} else if (mode == ShooterMode.DUMP_LOW) {
      setFlywheelSpeed(1750); // dump = 1750
    } else if (mode == ShooterMode.DUMP_HIGH) {
      setFlywheelSpeed(3500);*/
    //flywheelSpeedSetpoint = MathUtil.clamp(flywheelSpeedSetpoint, -6500, -1000);
    if (flywheelActive) {
      hoodTargetRPM = Math.abs(flywheelSpeedSetpoint) + speedOffset;
      double feedforwardOutput = hoodFeedforward.calculate(hoodTargetRPM) * 0.97;
      pid.setReference(flywheelSpeedSetpoint, ControlType.kVelocity);
      hoodRollers.set(ControlMode.Velocity, rpmToNativeUnits(hoodTargetRPM), DemandType.ArbitraryFeedForward, feedforwardOutput / nominalVoltage);
    } else {
      setFlywheelIdle();
    }
  }

}
