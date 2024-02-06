// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterSim {
    private final FlywheelSim shooterPhysicsSim;
    private final FlywheelSim hoodPhysicsSim;
    private final TalonFX shooter;
    private final TalonFX hood;
    private TalonFXSimState shooterSim;
    private TalonFXSimState hoodSim;

    private final MutableMeasure<Velocity<Angle>> shooterSpeed = RotationsPerSecond.zero().mutableCopy();
    private final MutableMeasure<Velocity<Angle>> hoodSpeed = RotationsPerSecond.zero().mutableCopy();

    public ShooterSim(TalonFX shooter, TalonFX hood) {
        shooterPhysicsSim = new FlywheelSim(
            DCMotor.getFalcon500(1), 
            1.0, 
            ShooterConstants.flywheelMOI
        );
        hoodPhysicsSim = new FlywheelSim(
            DCMotor.getFalcon500(1), 
            1.0, 
            ShooterConstants.hoodMOI
        );
        this.shooter = shooter;
        shooterSim = this.shooter.getSimState();
        this.hood = hood;
        hoodSim = this.hood.getSimState();
    }

    public double getShooterSimVel() {
        return shooterSpeed.in(RotationsPerSecond);
    }

    public double getHoodSimVel() {
        return hoodSpeed.in(RotationsPerSecond);
    }

    public void updateSim() {
        shooterSim = this.shooter.getSimState();
        hoodSim = this.hood.getSimState();

        // set input voltage so that it will account for voltage sag
        shooterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        hoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // update the wpilib physics model
        shooterPhysicsSim.setInputVoltage(shooterSim.getMotorVoltage());
        hoodPhysicsSim.setInputVoltage(hoodSim.getMotorVoltage());

        // step time
        shooterPhysicsSim.update(0.020);
        hoodPhysicsSim.update(0.020);

        // update motor sensors, compensate for units
        shooterSpeed.mut_replace(shooterPhysicsSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        hoodSpeed.mut_replace(hoodPhysicsSim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        shooterSim.setRotorVelocity(shooterSpeed.in(RotationsPerSecond));
        hoodSim.setRotorVelocity(hoodSpeed.in(RotationsPerSecond));
    }
}
