// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class AutoDriveTimed extends Command {
  private SwerveDrive swerveDrive;
  private Double time, xSpeed, ySpeed, turnSpeed, xtSpeed, ytSpeed, turntSpeed;
  private Double currentTime = 0.0;
  private ChassisSpeeds chassisSpeeds;
  private SlewRateLimiter xlimiter;
  private SlewRateLimiter ylimiter;
  private SlewRateLimiter tLimiter;

  /** Creates a new AutoDriveTimed. */
  public AutoDriveTimed(SwerveDrive swerveDrive, Double xSpeed, Double ySpeed, Double turnSpeed, Double time) {
    this.time = time;
    this.swerveDrive = swerveDrive;
    this.xtSpeed = xSpeed;
    this.ytSpeed = ySpeed;
    this.turntSpeed = turnSpeed;
    this.xlimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.ylimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.tLimiter = new SlewRateLimiter(DriveConstants.MaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xlimiter.calculate(xtSpeed);
    ySpeed = ylimiter.calculate(ytSpeed);
    turnSpeed = tLimiter.calculate(turntSpeed);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveDrive.getRotation2d());
    swerveDrive.drive(chassisSpeeds);
    currentTime += 0.02;
    if (currentTime >= time) {
      xtSpeed = 0.0;
      ytSpeed = 0.0;
      turntSpeed = 0.0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveDrive.getRotation2d());
    swerveDrive.drive(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentTime >= time + 0.2;
  }
}