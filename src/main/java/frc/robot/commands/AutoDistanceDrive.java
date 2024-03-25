// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class AutoDistanceDrive extends Command {
  private SwerveDrive swerveDrive;
  private Boolean ready = true;
  private Double xSpeed, ySpeed, turnSpeed, xtSpeed, ytSpeed, turntSpeed, dist = 0.0;
  private Double currentTime = 0.5;
  private ChassisSpeeds chassisSpeeds;
  private SlewRateLimiter xlimiter;
  private SlewRateLimiter ylimiter;
  private SlewRateLimiter tLimiter;

  /** Creates a new AutoDriveTimed. */
  public AutoDistanceDrive(SwerveDrive swerveDrive, Double dist, Double xSpeed, Double ySpeed, Double turnSpeed) {
    this.dist = dist;
    this.swerveDrive = swerveDrive;
    this.xtSpeed = xSpeed;
    this.ytSpeed = ySpeed;
    this.turntSpeed = turnSpeed;
    this.xSpeed = 0.0;
    this.ySpeed = 0.0;
    this.turnSpeed = 0.0;
    this.currentTime = 0.5;
    this.xlimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.ylimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.tLimiter = new SlewRateLimiter(DriveConstants.MaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.resetPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("chassisspeeds", xSpeed.toString()+", "+ySpeed.toString()+", "+turnSpeed.toString());
    SmartDashboard.putString("target", xtSpeed.toString()+", "+ytSpeed.toString()+", "+turntSpeed.toString());
    xSpeed = xlimiter.calculate(xtSpeed);
    ySpeed = ylimiter.calculate(ytSpeed);
    turnSpeed = tLimiter.calculate(turntSpeed);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveDrive.getRotation2d());
    swerveDrive.drive(chassisSpeeds);
    currentTime += 0.2;
    if (Math.abs(Math.sqrt((swerveDrive.getPose().getTranslation().getX() * swerveDrive.getPose().getTranslation().getX())
        + (swerveDrive.getPose().getTranslation().getY() * swerveDrive.getPose().getTranslation().getY()))) >= dist && ready) {
      xtSpeed = 0.0;
      ytSpeed = 0.0;
      turntSpeed = 0.0;
      currentTime = -0.4;
      ready = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveDrive.getRotation2d());
    swerveDrive.drive(chassisSpeeds);
    swerveDrive.resetPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
    swerveDrive.zeroHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentTime == 0.0;
  }
}