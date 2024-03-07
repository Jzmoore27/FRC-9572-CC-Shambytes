// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule frModule = new SwerveModule(
    SwerveConstants.frMotor0,
    SwerveConstants.frMotor1,
    true,
    true,
    SwerveConstants.frAbsEncoder,
    false,
    3);
  private SwerveModule flModule = new SwerveModule(
    SwerveConstants.flMotor0,
    SwerveConstants.flMotor1,
    true,
    true,
    SwerveConstants.flAbsEncoder,
    false,
    32);
  private SwerveModule brModule = new SwerveModule(
    SwerveConstants.brMotor0,
    SwerveConstants.brMotor1,
    true,
    false,
    SwerveConstants.brAbsEncoder,
    false,
    338);
  private SwerveModule blModule = new SwerveModule(
    SwerveConstants.blMotor0,
    SwerveConstants.blMotor1,
    true,
    true,
    SwerveConstants.blAbsEncoder,
    false,
    10);

  private final Pigeon2 gyro = new Pigeon2(0);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.DriveKinematics,
            new Rotation2d(0),new SwerveModulePosition[] {
              flModule.getPosition(),
              frModule.getPosition(),
              blModule.getPosition(),
              brModule.getPosition()
            });

  public SwerveDrive() {
    zeroHeading();

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void zeroHeading() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.setYaw(0);
      } catch (Exception e) {
      }
    }).start();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw().getValue(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

  public void resetPose(Pose2d pose) {
      odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{flModule.getPosition(),frModule.getPosition(),blModule.getPosition(),brModule.getPosition()}, pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.DriveKinematics.toChassisSpeeds(flModule.getState(),frModule.getState(),blModule.getState(),brModule.getState());
  }

  @Override
  public void periodic() {
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
              flModule.getPosition(),
              frModule.getPosition(),
              blModule.getPosition(),
              brModule.getPosition()
            });
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
}

  public void stopModules() {
    frModule.stop();
    flModule.stop();
    brModule.stop();
    blModule.stop();
  }

  public void setDesiredStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MaxMPS);
    frModule.setDesiredState(desiredStates[0]);
    flModule.setDesiredState(desiredStates[1]);
    brModule.setDesiredState(desiredStates[2]);
    blModule.setDesiredState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    DriveConstants.DriveKinematics.toSwerveModuleStates(chassisSpeeds);
  }
}