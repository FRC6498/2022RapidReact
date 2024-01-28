// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.NTHelper;
import static edu.wpi.first.units.Units.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  // motors
  private final WPI_TalonFX leftLeader, rightLeader;
  private final WPI_TalonFX leftFollower, rightFollower;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrive diffDrive;
  //
  private final Solenoid shifter; // gear shifter
  // imu
  private final AHRS gyro;
  
  private Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static boolean isHighGear = false;
  private boolean driveInverted;
  private NeutralMode currentBrakeMode = NeutralMode.Coast;
  private final SimpleMotorFeedforward drivetrainFeedforward =
    new SimpleMotorFeedforward(
      Constants.DriveConstants.kS,
      Constants.DriveConstants.kV,
      Constants.DriveConstants.kA
    );

  private DifferentialDrivePoseEstimator poseEstimator;
  Matrix<N5, N1> stateStdDevs;
  Matrix<N3, N1> localMeasurementStdDevs;
  Matrix<N3, N1> visionMeasurementStdDevs;

  public Drivetrain()
  {
    leftLeader = new WPI_TalonFX(DriveConstants.leftLeaderCANId);
    leftFollower = new WPI_TalonFX(DriveConstants.leftFollowerCANId);
    rightLeader = new WPI_TalonFX(DriveConstants.rightLeaderCANId);
    rightFollower = new WPI_TalonFX(DriveConstants.rightFollowerCANId);

    leftLeader.configOpenloopRamp(DriveConstants.driveRampRate);
    leftFollower.configOpenloopRamp(DriveConstants.driveRampRate);
    rightLeader.configOpenloopRamp(DriveConstants.driveRampRate);
    rightFollower.configOpenloopRamp(DriveConstants.driveRampRate);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
    rightMotors = new MotorControllerGroup(rightLeader, rightFollower);
    leftMotors.setInverted(true);
    diffDrive = new DifferentialDrive(leftMotors, rightMotors);
    diffDrive.setSafetyEnabled(false);

    gyro = new AHRS(Port.kMXP);
    gyro.reset();
    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);

    shifter = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.shifterChannelId);

    // engage brakes when neutral input
    setBrakeMode(NeutralMode.Brake);

    // setup encoders
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200);
    rightFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 199);
    leftFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 201);
    rightFollower.setStatusFramePeriod(StatusFrame.Status_1_General, 202);

    // state variable standard deviations - larger std dev -> increased uncertainty of state variables -> trust state less
    // state variables are        x pos, y pos,       heading,      left dist, right dist
    stateStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1), 0.02, 0.02);
    // sensor measurements standard deviations - larger std dev -> increased uncertainty of sensor measurements -> trust sensors less
    // sensor measurements are           left dist, right dist, gyro heading
    localMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(2));
    // vision measurements standard deviations - larger std dev -> increased uncertainty of vision measurements -> trust vision less
    // sensor measurements are               x pos, y pos, vision heading
    visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getGyroAngle(), getLeftDistanceMeters(), getRightDistanceMeters(), initialPose, localMeasurementStdDevs, visionMeasurementStdDevs);
  }

  // hardware methods

  public void resetSensors()
  {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
  }

  
  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  @Log(name = "Gyro Angle (deg.)")
  public double getGyroAngleDegrees() {
    return getGyroAngle().getDegrees();
  }

  public double getDistance(Pose2d endPose) {
    return initialPose.getTranslation().getDistance(endPose.getTranslation());
  }

  public void getInitialPose() {
    initialPose = getPose();
  }

  public void setBrakeMode(NeutralMode brakeMode)
  {
    leftLeader.setNeutralMode(brakeMode);
    rightLeader.setNeutralMode(brakeMode);
    currentBrakeMode = brakeMode;
  }

  public void setInverted(boolean inverted) {
    driveInverted = inverted;
  }

  public boolean getInverted() {
    return driveInverted;
  }

  public void toggleInverted() {
    driveInverted = !driveInverted;
  }

  public void arcadeDrive(double throttle, double turn) {
    if (driveInverted) {
      throttle *= -1;
      turn *= -1;
    }
    diffDrive.arcadeDrive(throttle, turn, true);
  }

  public void toggleGear()
  {
    if (isHighGear) {
      shifter.set(false);
      isHighGear = false;
    } else {
      shifter.set(true);
      isHighGear = true;
    }
  }

  //@Log(name="Gear", tabName = "SmartDashboard")
  public String getGear()
  {
    if (isHighGear) {
      return "High";
    } else {
      return "Low";
    }
  }

  public void stop()
  {
    arcadeDrive(0, 0);
  }

  //@Log
  public String getBrakeMode() {
    return currentBrakeMode.toString();
  }
  

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return drivetrainFeedforward;
  }

  /**
   * Resets odometry to a specified pose.
   * @param pose The pose to set the odometry pose to.
   */
  public void resetOdometry(Pose2d pose) {
    resetSensors();
    poseEstimator.resetPosition(getGyroAngle(), new DifferentialDriveWheelPositions(null, null), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public double getMeanEncoderDistance() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  @Override
  public void periodic() {
    poseEstimator.update(
      getGyroAngle(), 
      new DifferentialDriveWheelPositions(null, null)
    );

    NTHelper.setDouble("yaw_deg", getGyroAngleDegrees());
  }

  private double getLeftDistanceMeters() {
    return leftLeader.getSelectedSensorPosition() * DriveConstants.driveDistancePerTickMeters;
  }

  private double getRightDistanceMeters() {
    return rightLeader.getSelectedSensorVelocity() * DriveConstants.driveDistancePerTickMeters;
  }

  private double getLeftSpeedMetersPerSecond() {
    return leftLeader.getSelectedSensorVelocity() * DriveConstants.driveDistancePerTickMeters * 10;
  }

  private double getRightSpeedMetersPerSecond() {
    return rightLeader.getSelectedSensorVelocity() * DriveConstants.driveDistancePerTickMeters * 10;
  }
}
