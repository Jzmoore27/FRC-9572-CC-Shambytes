// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class SwerveModule extends SubsystemBase {

  private PIDController turningPIDController;
  private AnalogInput absoluteEncoder;
  private BaseMotorController driveMotor; 
  private BaseMotorController turningMotor;
  private double absoluteEncoderOffset;
  private boolean absoluteEncoderReversed;

  /** Creates a new SwerveModule. */
  public SwerveModule(BaseMotorController turningMotor, BaseMotorController driveMotor, boolean turningMotorReversed,
      boolean driveMotorReversed, AnalogInput absoluteEncoder, boolean absoluteEncoderReversed,
      double absoluteEncoderOffset) {

    // Reverse Motors
    this.driveMotor.setInverted(driveMotorReversed);
    this.turningMotor.setInverted(turningMotorReversed);

    // initialize PIDController
    turningPIDController = new PIDController(SwerveConstants.PIDProportional, 0, 0);
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // initialize Global Variables
    absoluteEncoder = this.absoluteEncoder;
    absoluteEncoderOffset = this.absoluteEncoderOffset;
    absoluteEncoderReversed = this.absoluteEncoderReversed;
    driveMotor = this.driveMotor;
    turningMotor = this.turningMotor;

    // reset Encoders
    resetEncoders();
  }

  // Return Methods
  public double getDrivePosition() {
    return (driveMotor.getSelectedSensorPosition() / (512 / 90)) * (SwerveConstants.driveWheelDiameterMeters / 360);
  }
  public double getTurningPosition() {
    return (turningMotor.getSelectedSensorPosition() / (512 / 90)) * (1 / 360);
  }

  public double getDriveSpeed() {
    return (driveMotor.getSelectedSensorVelocity() / (512 / 90)) * (SwerveConstants.driveWheelDiameterMeters / 360);
  }

  public double getTurningSpeed() {
    return Math.toRadians(turningMotor.getSelectedSensorVelocity() / (512 / 90));
  }

  private double getAbsEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffset;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  // Stop Motors
  private void stop() {
    driveMotor.set(ControlMode.PercentOutput,0);
    turningMotor.set(ControlMode.PercentOutput,0);
  }

  // Reset Encoders
  private void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(Math.toDegrees(getAbsEncoderRad()) * (512 / 90)); //DONE-- Change to return 0 to 2048 and not radians
  }

  // State handling
  private SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getTurningPosition())); //DONE-- Change to Return in radians
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.MaxMPS);
    turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}