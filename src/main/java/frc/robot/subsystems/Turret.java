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
import frc.robot.Constants.TurretConstants;
import frc.robot.lib.NTHelper;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
/**
 * THIS SUBSYSTEM IS WIP AND CURRENTLY UNUSED BY THE MAIN ROBOT CODE. IT WILL REMAIN SO UNTIL IT IS FINISHED.<p>
 * Turret subsystem using WPILib controls instead of Phoenix (because its less of a black box and the units are better).
 * Position is measured in Degrees, which is obtained from encoder ticks as soon as position is read off.
 */
// turret clockwise = forward 
public class Turret extends SubsystemBase implements Loggable {
  WPI_TalonFX bearing;
  TalonFXConfiguration bearingConfig;
  public boolean homed;
  @Log.Graph(name = "Yaw Angle (deg.)")
  double visionDegrees;
  @Log.Graph
  double pidOutput;
  SimpleMotorFeedforward turretFeedforward;
  @Log.ToString(name = "Turret Mode", tabName = "SmartDashboard")
  ShooterMode mode;
  private Rotation2d turretPositionSetpoint;
  private Rotation2d turretCurrentPosition;

  public Turret() {
    mode = ShooterMode.DISABLED;
    visionDegrees = 0.0;
    pidOutput = 0.0;
    homed = false;
    bearing = new WPI_TalonFX(TurretConstants.yawMotorCANId);
    bearingConfig = new TalonFXConfiguration();
    bearingConfig.peakOutputForward = 0.2;
    bearingConfig.peakOutputReverse = -0.2;
    bearingConfig.slot0.kP = TurretConstants.kP;
    bearingConfig.slot0.kI = 0;
    bearingConfig.slot0.kD = TurretConstants.kD;
    bearingConfig.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    bearingConfig.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    bearingConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    bearing.configAllSettings(bearingConfig);
    bearing.setNeutralMode(NeutralMode.Brake);
    // set position tolerance to 1 degree
    turretFeedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    turretCurrentPosition = Rotation2d.fromDegrees(0);
    turretPositionSetpoint = Rotation2d.fromDegrees(0);
  }

  public void setSetpointDegrees(double setpoint) {
    visionDegrees = setpoint;
  }

  private void useOutput() {
    bearing.set(TalonFXControlMode.Position, -rotation2dToNativeUnits(turretPositionSetpoint), DemandType.ArbitraryFeedForward, turretFeedforward.calculate(turretPositionSetpoint.getDegrees()));
  }

  private double rotation2dToNativeUnits(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    return degrees * -254.7;
  }

  private Rotation2d nativeUnitsToRotation2d(double units) {
    //254.7 ticks / 1 degree
    return Rotation2d.fromDegrees(units / -254.7);
  }

  public void stop() {
    bearing.set(0);
  }

  public boolean atSetpoint() {
    return nativeUnitsToRotation2d(bearing.getClosedLoopError()).getDegrees() < 1;
  }

  public boolean getActive() {
    // return mode < 2
    return mode != ShooterMode.DUMP || mode != ShooterMode.DISABLED;
  }

  @Override
  public void periodic() {
    turretCurrentPosition = nativeUnitsToRotation2d(bearing.getSelectedSensorPosition());
    NTHelper.setString("turret_shooter_mode", mode.toString());
    NTHelper.setDouble("turret_position_deg", turretCurrentPosition.getDegrees());
    NTHelper.setDouble("turret_setpoint_deg", turretPositionSetpoint.getDegrees());
    switch (mode) {
      case FULL_AUTO:
      case MANUAL_FIRE:
        useOutput();
      case DUMP:
      case DISABLED:
        //bearing.setVoltage(0);
        break;
      case HOMING:
        useOutput();
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

  public void setInverted() {
    bearing.setInverted(true);
  }

  public void setForward() {
    bearing.setInverted(false);
  }

  public void resetSensor() {
    bearing.setSelectedSensorPosition(rotation2dToNativeUnits(TurretConstants.maxCounterClockwise));
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
