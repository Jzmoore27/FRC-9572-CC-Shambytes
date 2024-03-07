// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.SwerveDriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class SwerveModule extends SubsystemBase {

  private PIDController turningPIDController;
  private CANcoder absoluteEncoder;
  private TalonFX driveMotor;
  private TalonFX turningMotor;
  private double absoluteEncoderOffset;

  /** Creates a new SwerveModule. */
  public SwerveModule(TalonFX turningMotor, TalonFX driveMotor, boolean turningMotorReversed,
      boolean driveMotorReversed, CANcoder absoluteEncoder, boolean absoluteEncoderReversed,
      double absoluteEncoderOffset) {

    // Reverse Motors
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    // initialize PIDController
    turningPIDController = new PIDController(SwerveConstants.PIDProportional, 0, 0.01);
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // initialize Global Variables
    this.absoluteEncoder = absoluteEncoder;
    this.driveMotor = driveMotor;
    this.turningMotor = turningMotor;
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    // reset Encoders
    resetEncoders();
  }

  // Return Methods
  public double getDrivePosition() {
    return (driveMotor.getPosition().getValue() * SwerveConstants.driveWheelCircumferenceMeters);
  }

  public double getTurningPosition() {
    return (turningMotor.getPosition().getValue()*(7/150.0));
  }

  public double getDriveSpeed() {
    return (driveMotor.getVelocity().getValue() * SwerveConstants.driveWheelCircumferenceMeters);
  }

  public double getTurningSpeed() {
    return Math.toRadians(turningMotor.getVelocity().getValue()*(7/150.0));
  }

  private double getAbsEncoderRad() {
    if((absoluteEncoder.getAbsolutePosition().getValue() * 360) + absoluteEncoderOffset > 360){
      return Math.toRadians((absoluteEncoder.getAbsolutePosition().getValue() * 360) + absoluteEncoderOffset - 360);
    }
    else{
      return Math.toRadians((absoluteEncoder.getAbsolutePosition().getValue() * 360) + absoluteEncoderOffset);
    }
  }

  // Stop Motors
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // Reset Encoders
  private void resetEncoders() {
    driveMotor.setPosition(0);
    turningMotor.setPosition((Math.toDegrees(getAbsEncoderRad())/360)*(150/7.0));
    // DONE-- Change to return
    // 0 to 2048 and not
    // radians
  }

  // State handling
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getAbsEncoderRad()));
    // DONE-- Change to Return in
    // radians
  }

  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getState().angle);
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.MaxMPS);
    if(Math.abs(getAbsEncoderRad() - state.angle.getRadians())<0.001){
      turningMotor.set(0);
      return;
    }
    turningMotor.set(turningPIDController.calculate(getAbsEncoderRad(), state.angle.getRadians()));
    }
  
  public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getAbsEncoderRad())
        );
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("abs:" + absoluteEncoder.getDeviceID(), getAbsEncoderRad());
  }
}