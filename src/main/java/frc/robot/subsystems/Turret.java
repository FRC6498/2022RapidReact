// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import frc.robot.Constants.TurretConstants;
import frc.robot.lib.NTHelper;
import frc.robot.subsystems.Superstructure.ShooterMode;
import monologue.Logged;
import monologue.Annotations.Log;

// turret clockwise = forward 
public class Turret extends SubsystemBase implements Logged {
  private boolean homed;
  private ShooterMode mode;
  private Rotation2d turretPositionSetpoint;
  private Rotation2d turretCurrentPosition;
  private Supplier<OptionalDouble> targetYaw;
  private TalonFX bearing;
  private TalonFXConfiguration bearingConfig;
  private DutyCycleOut percentOut = new DutyCycleOut(0);
  private PositionVoltage position = new PositionVoltage(0);

  public Trigger fwdLimit, revLimit;

  public Turret(Supplier<OptionalDouble> targetYaw) {
    bearing = new TalonFX(TurretConstants.yawMotorCANId);
    bearingConfig = new TalonFXConfiguration();
    bearingConfig.MotorOutput.PeakForwardDutyCycle = 0.3;
    bearingConfig.MotorOutput.PeakReverseDutyCycle = -0.3;
    bearingConfig.Slot0.kP = TurretConstants.kP;
    bearingConfig.Slot0.kI = TurretConstants.kI;
    bearingConfig.Slot0.kD = TurretConstants.kD;
    bearingConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    bearingConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    bearingConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    bearingConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    // set position tolerance to 200 ticks (1 deg ~ 359)
    //bearingConfig.slot0.allowableClosedloopError = rotation2dToNativeUnits(Rotation2d.fromDegrees(TurretConstants.turretPositionToleranceDegrees));
    bearingConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    bearingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bearingConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    bearingConfig.Feedback.SensorToMechanismRatio = TurretConstants.turretRotorToMechanismRatio;
    bearing.getConfigurator().apply(bearingConfig);
    
    turretCurrentPosition = Rotation2d.fromDegrees(0);
    turretPositionSetpoint = Rotation2d.fromDegrees(0);

    fwdLimit = new Trigger(this::getFwdLimit);
    revLimit = new Trigger(this::getRevLimit);
    this.targetYaw = targetYaw;
    setDefaultCommand(Commands.idle(this));
  }

  public Command stop() {
    return runOnce(() -> bearing.set(0))
    .andThen(Commands.idle(this));
  }

  public Command track() {
    return run(() -> {
      // subtract the yaw of the target if we see one, or else subtract nothing
      Rotation2d angle = getCurrentPosition().minus(Rotation2d.fromDegrees(targetYaw.get().orElse(0)));
      NTHelper.setDouble("turret_setpoint_calc", angle.getDegrees());
      setPositionSetpoint(angle);
    });
  }

  public boolean atSetpoint() {
    return bearing.getClosedLoopError().getValue() < TurretConstants.turretPositionToleranceDegrees.in(Rotations);
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
  }

  public Command home() {
    return Commands.sequence(
      runOnce(() -> setShooterMode(ShooterMode.HOMING)),
      runOnce(() -> bearing.setControl(percentOut.withOutput(0.15))),
      Commands.waitUntil(this::checkLimits),
      runOnce(() -> bearing.setControl(percentOut.withOutput(0))),
      runOnce(() -> setPositionSetpoint(TurretConstants.frontDumpAngle)),
      runOnce(() -> setShooterMode(ShooterMode.MANUAL_FIRE))
    );
  }

  public boolean getFwdLimit() {
    return bearing.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getRevLimit() {
    return bearing.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public boolean checkLimits() {
    NTHelper.setBoolean("homed", homed);
    if (getFwdLimit()) {
      reset(TurretConstants.hardForwardAngle);
      NTHelper.setBoolean("forward_limit", true);
      NTHelper.setBoolean("reverse_limit", false);
      return true;
    } else if (getRevLimit()) {
      reset(TurretConstants.hardReverseAngle);
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
    bearing.setPosition(angle.getRotations());
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
    bearing.setControl(position.withPosition(setpoint.getRotations()));
  }

  
  public Rotation2d getCurrentPosition() {
    turretCurrentPosition = Rotation2d.fromRotations(bearing.getPosition().getValue());
    return turretCurrentPosition;
  }
  
  @Log //(name = "Turret Position (deg.)")
  public double getCurrentPositionDegrees() {
    return getCurrentPosition().getDegrees();
  }
}
