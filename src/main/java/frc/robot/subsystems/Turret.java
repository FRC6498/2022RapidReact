// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Turret extends SubsystemBase implements Loggable {

  TalonFX yawMotor = new TalonFX(yawMotorCANId);
  Rotation2d angleGoal = Rotation2d.fromDegrees(135);
  double openLoopDemand;
  public boolean enabled = true;
  double setAbsolutePositionTicks;
  Timer velocityTimer;
  /** Creates a new Turret. */
  public Turret() {
    yawMotor.configFactoryDefault();
    yawMotor.configPeakOutputForward(0.2);
    yawMotor.configPeakOutputReverse(-0.2);
    yawMotor.config_kP(0, turretYaw_kP);
    yawMotor.config_kI(0, 0);
    yawMotor.config_kD(0, turretYaw_kD);
    //yawMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    yawMotor.setNeutralMode(NeutralMode.Coast);

    velocityTimer = new Timer();
  }

  public void resetSensors(double position) {
    yawMotor.setSelectedSensorPosition(position);
  }

  @Log
  public double getYawMotorOutputCurrent() {
    return yawMotor.getStatorCurrent();
  }

  @Log
  public double getPositionError() {
    return yawMotor.getClosedLoopError();
  }

  public boolean atSetpoint() {
    return getPositionError() < turretPositionOffsetThreshold;
  }

  public double getYawVelocityDegreesPerSecond() {
    return encoderTicksToDegrees(yawMotor.getSelectedSensorVelocity());
  }

  @Log(name = "Open Loop Demand")
  public double getOpenLoopDemand() {
    return openLoopDemand;
  }

  public void openLoop(double demand) {
    openLoopDemand = demand;
    yawMotor.set(ControlMode.PercentOutput, demand);
    //DriverStation.reportWarning(yawMotor.getLastError().toString(), false);
  }

  public void setAbsoluteAngleGoal(Rotation2d angle) {
    double absolutePositionDegrees = angle.getDegrees();
    double absolutePositionTicks = degreesToEncoderTicks(absolutePositionDegrees);
    yawMotor.set(ControlMode.Position, absolutePositionTicks);
  }

  public void setRelativeAngleGoal(Rotation2d angle) {
    // convert to absolute
    double absAngleDegrees = getAngle().getDegrees() + angle.getDegrees();
    setAbsoluteAngleGoal(Rotation2d.fromDegrees(absAngleDegrees));
    
  }

  public void reset(Rotation2d angle) {
    yawMotor.setSelectedSensorPosition((angle.getRadians() / 2 * Math.PI) * turretTicksPerRotation);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Units.radiansToDegrees(yawMotor.getSelectedSensorPosition() / turretTicksPerRotation * 2 * Math.PI));
  }

  public void setSoftLimitsEnable(boolean enable) {
    enable = false;
    if (enable) {
      yawMotor.configForwardSoftLimitThreshold(turretMaxPosition - turretSoftLimitOffset);
      yawMotor.configReverseSoftLimitThreshold(turretMinPosition + turretSoftLimitOffset);
    } else {
      yawMotor.configSoftLimitDisableNeutralOnLOS(true, 50);
    }
  }

  public void turretTurn() {
    yawMotor.set(ControlMode.Position, setAbsolutePositionTicks);
}
//I might need this later
 //public double checkTheNumbers() {
  // return setAbsolutePositionTicks;
 //}
  public void setEnabled(boolean enabled) { this.enabled = enabled; }

  public boolean getEnabled() { return enabled; }

  @Log(name = "Angle Goal (deg)")
  public double getAngleGoalDegrees() {
    return angleGoal.getDegrees();
  }
  @Log(name = "Current Angle (deg)")
  public double getCurrentAngleDegrees() {
    return getAngle().getDegrees();
  }
}
