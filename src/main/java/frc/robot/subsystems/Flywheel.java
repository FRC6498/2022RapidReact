// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.lib.InterpolatingTable;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ShooterConstants;

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
  private double speedOffset = 0;
  @Log
  private double hoodTargetRPM = 0;
  private TalonFXConfiguration hoodConfig;

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
      0.86936, 
      0.11827, 
      0.0060958
    );
    pid = neo.getPIDController();
    
    hoodRollers = new WPI_TalonFX(ShooterConstants.hoodRollerCANId);
    hoodConfig = new TalonFXConfiguration();
    hoodConfig.slot0.kP = 0.076642;
    hoodConfig.slot0.kI = 0;
    hoodConfig.slot0.kD = 0;
    hoodConfig.slot0.kF = 0;
    // once we hit 40A for >=100ms, hold at 40A
    hoodConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 100);
    hoodRollers.configAllSettings(hoodConfig);
    neo.enableVoltageCompensation(12.3);
    hoodRollers.configVoltageCompSaturation(12.3);
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
      case TUNING:
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

  private double rpmToFalconTicks(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  private double falconTicksToRPM(double ticks) {
    return ticks / 2048.0 * 600.0;
  }

  @Override
  public void periodic() {
    switch (mode) {
      case MANUAL_FIRE:
        //setFlywheelSpeed(InterpolatingTable.get(distanceToHub).rpm);
        //setFlywheelSpeed(3500);
        break;
      default:
        break;
    }
      
    /*} else if (mode == ShooterMode.DUMP_LOW) {
      setFlywheelSpeed(1750); // dump = 1750
    } else if (mode == ShooterMode.DUMP_HIGH) {
      setFlywheelSpeed(3500);*/
    //flywheelSpeedSetpoint = MathUtil.clamp(flywheelSpeedSetpoint, -6500, -1000);
    if (flywheelActive) {
      hoodTargetRPM = rpmToFalconTicks(Math.abs(flywheelSpeedSetpoint) + speedOffset);
      pid.setReference(flywheelSpeedSetpoint, ControlType.kVelocity);
      hoodRollers.set(ControlMode.Velocity, hoodTargetRPM, DemandType.ArbitraryFeedForward, hoodFeedforward.calculate(hoodTargetRPM));
    } else {
      setFlywheelIdle();
    }
  }

}
