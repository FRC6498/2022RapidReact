// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.InterpolatingTable;
import frc.robot.lib.NTHelper;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends SubsystemBase implements Loggable {
  // Hardware
  private final CANSparkMax neo;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  // Software
  //private final SimpleMotorFeedforward flywheelFeedforward;
  private boolean flywheelActive;
  public double flywheelSpeedSetpoint = -3000.0;
  public double flywheel_speed_target;
  private int speedOffset = 0;

  //private double feedforwardOutput;
  double lastPosition = 0.0;
  double distanceToHub = 0.0;
  //@Log.ToString(name = "Flywheel Mode", tabName = "SmartDashboard")
  private ShooterMode mode;
  
  public Flywheel() {
    mode = ShooterMode.DISABLED;
    neo = new CANSparkMax(flywheelCANId, MotorType.kBrushless);
    encoder = neo.getEncoder();
    //flywheelFeedforward = new SimpleMotorFeedforward(
    //  flywheelkS, 
    //  flywheelkV, 
    //  flywheelkA
    //);
    pid = neo.getPIDController();
    
    //neo.restoreFactoryDefaults(true);
    //neo.setIdleMode(IdleMode.kCoast);
    neo.enableVoltageCompensation(12.3);
  } 
  
  /**
   * 
   * @param velocity Desired velocity in rpm
   */
  //@Config(name = "Set Flywheel Speed(RPM)")
  public void setFlywheelSpeed(double velocity) {
      flywheelSpeedSetpoint = -(velocity + speedOffset);
  }

  public void setFlywheelDistance(double distance) {
    distanceToHub = distance;
  }

  public void incrementOffset() {
    speedOffset += 100;
  }

  public void decrementOffset() {
    speedOffset -= 100;
  }

  //@Log(name = "Flywheel Velocity (RPM)")
  public double getFlywheelSpeed() {
    return encoder.getVelocity();
  }

  public void setFlywheelIdle() {
    pid.setReference(0, ControlType.kDutyCycle);
  }

  public boolean getActive() {
    return mode != ShooterMode.DISABLED || mode != ShooterMode.HOMING;
  }

  public boolean atSetpoint() {
    return Math.abs(getFlywheelSpeed() - flywheelSpeedSetpoint) < flywheelSetpointToleranceRPM;
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
    switch (mode) {
      case FULL_AUTO:
      case MANUAL_FIRE:
      case DUMP:
        flywheelActive = true;
        break;
      case DISABLED:
        flywheelActive = false;
        break;
      default:
        break;
    }
  }
  
  @Override
  public void periodic() {
    if (mode == ShooterMode.MANUAL_FIRE) {
      setFlywheelSpeed(InterpolatingTable.get(distanceToHub).rpm);
    } else if (mode == ShooterMode.DUMP) {
      setFlywheelSpeed(1750); // check this
    } else if (mode == ShooterMode.DISABLED) {
      pid.setReference(0.0, ControlType.kDutyCycle);
    }
    flywheelSpeedSetpoint = MathUtil.clamp(flywheelSpeedSetpoint, -6500, -1000);
    flywheel_speed_target = flywheelSpeedSetpoint;
    NTHelper.setDouble("flywheel_speed_target", flywheelSpeedSetpoint);
    NTHelper.setDouble("flywheel_speed_actual", getFlywheelSpeed());
    if (flywheelActive) {
      pid.setReference(flywheelSpeedSetpoint, ControlType.kVelocity);
    } else {
      setFlywheelIdle();
    }
  }

}
