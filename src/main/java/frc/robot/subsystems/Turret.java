// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
import frc.robot.lib.NTHelper;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
// turret clockwise = forward 
public class Turret extends SubsystemBase implements Loggable {
  private WPI_TalonFX bearing;
  private TalonFXConfiguration bearingConfig;
  private boolean homed;
  private boolean isHoming;
  private ShooterMode mode;
  private Rotation2d turretPositionSetpoint;
  private Rotation2d turretCurrentPosition;
  public Trigger fwdLimit, revLimit;

  public Turret() {
    mode = ShooterMode.DISABLED;
    homed = false;
    isHoming = false;
    bearing = new WPI_TalonFX(TurretConstants.yawMotorCANId);
    bearingConfig = new TalonFXConfiguration();
    bearingConfig.peakOutputForward = 0.3;
    bearingConfig.peakOutputReverse = -0.3;
    bearingConfig.slot0.kP = TurretConstants.kP;
    bearingConfig.slot0.kI = TurretConstants.kI;
    bearingConfig.slot0.kD = TurretConstants.kD;
    bearingConfig.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    bearingConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    // set position tolerance to 200 ticks (1 deg ~ 359)
    bearingConfig.slot0.allowableClosedloopError = rotation2dToNativeUnits(Rotation2d.fromDegrees(TurretConstants.turretPositionToleranceDegrees));
    bearingConfig.slot0.integralZone = 1023/bearingConfig.slot0.kP;
    bearingConfig.closedloopRamp = 0.1;
    bearing.configAllSettings(bearingConfig);
    bearing.setNeutralMode(NeutralMode.Brake);
    bearing.setInverted(TalonFXInvertType.CounterClockwise);
    
    turretCurrentPosition = Rotation2d.fromDegrees(0);
    turretPositionSetpoint = Rotation2d.fromDegrees(0);

    fwdLimit = new Trigger(this::getFwdLimit);
    revLimit = new Trigger(this::getRevLimit);
  }

  private double rotation2dToNativeUnits(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    return degrees * TurretConstants.turretTicksPerDegree;
  }

  private Rotation2d nativeUnitsToRotation2d(double units) {
    //254.7 ticks / 1 degree
    return Rotation2d.fromDegrees(units / TurretConstants.turretTicksPerDegree);
  }

  public void stop() {
    bearing.set(0);
  }

  public boolean atSetpoint() {
    return nativeUnitsToRotation2d(bearing.getClosedLoopError()).getDegrees() < TurretConstants.turretPositionToleranceDegrees;
  }

  public boolean getActive() {
    // return mode < 2
    return mode == ShooterMode.MANUAL_FIRE;
  }

  @Override
  public void periodic() {
    getCurrentPosition();
    NTHelper.setString("turret_shooter_mode", mode.toString());
    NTHelper.setDouble("turret_position_deg", turretCurrentPosition.getDegrees());
    NTHelper.setDouble("turret_setpoint_deg", turretPositionSetpoint.getDegrees());
    NTHelper.setBoolean("turret_at_setpoint", atSetpoint());
    
    if (!homed && !isHoming) {
      startHome();
    }
    if (!homed) {
      home();
    }
    
    checkLimits();

  }

  public void startHome() {
    homed = false;
    isHoming = true;
    // counter clockwise, this is positive if motor is uninverted
    openLoop(0.15);
    bearing.overrideSoftLimitsEnable(false);
  }

  public boolean getHomed() {
    return homed;
  }

  public void home() {
    if (checkLimits()) {
      openLoop(0);
      homed = true;
      bearing.overrideSoftLimitsEnable(false);
    }
  }

  public void openLoop(double percent) {
    bearing.set(ControlMode.PercentOutput, percent);
  }

  public boolean getFwdLimit() {
    return bearing.isFwdLimitSwitchClosed() == 1;
  }

  public boolean getRevLimit() {
    return bearing.isRevLimitSwitchClosed() == 1;
  }

  public boolean checkLimits() {
    NTHelper.setBoolean("homed", homed);
    if (getFwdLimit()) {
      reset(Rotation2d.fromDegrees(TurretConstants.hardForwardAngle));
      NTHelper.setBoolean("forward_limit", true);
      NTHelper.setBoolean("reverse_limit", false);
      return true;
    } else if (getRevLimit()) {
      reset(Rotation2d.fromDegrees(TurretConstants.hardReverseAngle));
      NTHelper.setBoolean("reverse_limit", true);
      NTHelper.setBoolean("forward_limit", false);
      return true;
    } else {
      NTHelper.setBoolean("forward_limit", false);
      NTHelper.setBoolean("reverse_limit", false);
      return false;
    }
  }

  private void reset(Rotation2d angle) {
    bearing.setSelectedSensorPosition(rotation2dToNativeUnits(angle));
    turretCurrentPosition = angle;
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
  }

  public void setInverted() {
    bearing.setInverted(true);
  }

  public void setForward() {
    bearing.setInverted(false);
  }

  public void setAngleRelative(double degrees) {
    Rotation2d transform = Rotation2d.fromDegrees(degrees);
    Rotation2d pose = turretPositionSetpoint.plus(transform);
    setPositionSetpoint(pose);
  }

  public void setPositionSetpoint(Rotation2d setpoint) {
    //System.out.println(setpoint.getDegrees());
    turretPositionSetpoint = setpoint;
    bearing.set(ControlMode.Position, rotation2dToNativeUnits(setpoint));
  }

  
  public Rotation2d getCurrentPosition() {
    turretCurrentPosition = nativeUnitsToRotation2d(bearing.getSelectedSensorPosition());
    return turretCurrentPosition;
  }
  
  @Log(name = "Turret Position (deg.)")
  public double getCurrentPositionDegrees() {
    return getCurrentPosition().getDegrees();
  }
}
