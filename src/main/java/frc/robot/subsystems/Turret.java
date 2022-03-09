// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.TurretConstants.*;

import java.util.function.DoubleSupplier;
/**
 * THIS SUBSYSTEM IS WIP AND CURRENTLY UNUSED BY THE MAIN ROBOT CODE. IT WILL REMAIN SO UNTIL IT IS FINISHED.<p>
 * Turret subsystem using WPILib controls instead of Phoenix (because its less of a black box and the units are better).
 * Position is measured in Degrees, which is obtained from encoder ticks as soon as position is read off.
 */
public class Turret extends SubsystemBase implements Loggable {
  WPI_TalonFX bearing;
  TalonFXConfiguration bearingConfig;
  PIDController pid;
  boolean homed;
  @Log.Graph(name = "Yaw Angle (deg.)")
  double visionDegrees;
  @Log.Graph
  double pidOutput;
  SimpleMotorFeedforward turretFeedforward;
  @Log.ToString(name = "Turret Mode", tabName = "SmartDashboard")
  ShooterMode mode;
  private Rotation2d turretPositionSetpoint;

  public Turret() {
    mode = ShooterMode.DISABLED;
    visionDegrees = 0.0;
    pidOutput = 0.0;
    homed = false;
    bearing = new WPI_TalonFX(yawMotorCANId);
    bearingConfig = new TalonFXConfiguration();
    bearingConfig.peakOutputForward = 0.2;
    bearingConfig.peakOutputReverse = -0.2;
    bearingConfig.slot0.kP = turretYaw_kP;
    bearingConfig.slot0.kI = 0;
    bearingConfig.slot0.kD = turretYaw_kD;
    bearingConfig.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    bearingConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    bearing.configAllSettings(bearingConfig);
    bearing.setNeutralMode(NeutralMode.Brake);
    pid = new PIDController(turretYaw_kP, 0, turretYaw_kD);
    // set position tolerance to 1 degree
    pid.setTolerance(turretPositionToleranceDegrees);
    turretFeedforward = new SimpleMotorFeedforward(turretFeedforward_kA, turretFeedforward_ks, turretFeedforward_kv);
    
  }

  public void setSetpointDegrees(double setpoint) {
    visionDegrees = setpoint;
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  private void useOutput() {
    bearing.set(TalonFXControlMode.Position, rotation2dToNativeUnits(turretPositionSetpoint), DemandType.Neutral, 0);
  }

  private double rotation2dToNativeUnits(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    return degrees * 178.352;
  }

  public void stop() {
    bearing.set(0);
  }

  public boolean getActive() {
    // return mode < 2
    return mode != ShooterMode.DUMP || mode != ShooterMode.DISABLED;
  }

  @Override
  public void periodic() {
    switch (mode) {
      case FULL_AUTO:
      case MANUAL_FIRE:
        useOutput();
      case DUMP:
      case DISABLED:
        //bearing.setVoltage(0);
        break;
      default:
        break;
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

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
  }

  public void resetSensor() {
    bearing.setSelectedSensorPosition(0);
  }

  public void setAngleRelative(double degrees) {
    Rotation2d transform = Rotation2d.fromDegrees(degrees);
    Rotation2d pose = turretPositionSetpoint.plus(transform);
    setPositionSetpoint(pose);
  }

  public void setPositionSetpoint(Rotation2d setpoint) {
    turretPositionSetpoint = setpoint;
  }
}
