// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.BasePigeon;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule frModule = new SwerveModule(
      SwerveConstants.frMotor0,
      SwerveConstants.frMotor1,
      false,
      false,
      SwerveConstants.frAbsEncoder,
      false,
      0);
  private SwerveModule flModule = new SwerveModule(
      SwerveConstants.flMotor0,
      SwerveConstants.flMotor1,
      false,
      false,
      SwerveConstants.flAbsEncoder,
      false,
      0);
  private SwerveModule brModule = new SwerveModule(
      SwerveConstants.brMotor0,
      SwerveConstants.brMotor1,
      false,
      false,
      SwerveConstants.brAbsEncoder,
      false,
      0);
  private SwerveModule blModule = new SwerveModule(
      SwerveConstants.blMotor0,
      SwerveConstants.blMotor1,
      false,
      false,
      SwerveConstants.blAbsEncoder,
      false,
      0);

  private final BasePigeon gyro = new BasePigeon(0, "");

  public SwerveDrive() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();
  }

  private void zeroHeading() {
    gyro.setYaw(0);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    frModule.stop();
    flModule.stop();
    brModule.stop();
    blModule.stop();
  }

  public void setDesiredStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MaxMPS);
    frModule.setDesiredState(desiredStates[1]);
    blModule.setDesiredState(desiredStates[2]);
    brModule.setDesiredState(desiredStates[3]);
  }

}