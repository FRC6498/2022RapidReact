// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterConstants.RotationsPerMinute;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterSim {
    private final FlywheelSim shooterPhsyicsSim;
    private final FlywheelSim hoodPhysicsSim;
    private final TalonFX shooter;
    private final TalonFX hood;
    private final TalonFXSimState shooterSim;
    private final TalonFXSimState hoodSim;

    private final MutableMeasure<Velocity<Angle>> shooterSpeed = RotationsPerSecond.zero().mutableCopy();

    public ShooterSim(TalonFX shooter, TalonFX hood) {
        shooterPhsyicsSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(
            ShooterConstants.flywheelkV, 
            ShooterConstants.flywheelkA
            ), DCMotor.getFalcon500(1), 1
        );
        hoodPhysicsSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(12.0 / 6380.0 / (2 * Math.PI), 0.001 / 6.28),
            DCMotor.getFalcon500(1), 
            1
        );
        this.shooter = shooter;
        shooterSim = this.shooter.getSimState();
        this.hood = hood;
        hoodSim = this.hood.getSimState();
    }

    public double getShooterSimVelRPM() {
        return shooterSpeed.mut_replace(shooterPhsyicsSim.getAngularVelocityRadPerSec(), RadiansPerSecond).in(RotationsPerMinute);
    }

    public void updateSim() {
        // set input voltage so that it will account for voltage sag
        shooterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        hoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // update the wpilib physics model
        shooterPhsyicsSim.setInputVoltage(shooterSim.getMotorVoltage());
        hoodPhysicsSim.setInputVoltage(hoodSim.getMotorVoltage());

        // step time
        shooterPhsyicsSim.update(0.020);
        hoodPhysicsSim.update(0.020);

        // update motor sensors, compensate for units
        shooterSpeed.mut_replace(shooterPhsyicsSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        shooterSim.setRotorVelocity(shooterSpeed.in(RotationsPerSecond));
        //hoodSim.setRotorVelocity(Units.radiansToRotations(hoodPhysicsSim.getAngularVelocityRadPerSec()));
    }
}
