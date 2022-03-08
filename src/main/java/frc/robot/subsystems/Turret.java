// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterMode;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.TurretConstants.*;
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
  ShooterMode mode;

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
    pidOutput = pid.calculate(visionDegrees, 0);
    bearing.setVoltage(pidOutput);
  }

  public void stop() {
    bearing.set(0);
  }

  
  @Override
  public void periodic() {
    switch (mode) {
      case FULL_AUTO:
      case MANUAL_FIRE:
        useOutput();
      case DUMP:
      case DISABLED:
        bearing.setVoltage(0);
        break;
      default:
        break;
    }
  }

  public void setShooterMode(ShooterMode mode) {
    this.mode = mode;
  }
}
