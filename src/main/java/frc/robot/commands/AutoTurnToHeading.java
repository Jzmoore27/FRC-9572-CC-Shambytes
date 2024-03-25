// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants.DriveConstants;
import frc.robot.Constants.SwerveDriveConstants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class AutoTurnToHeading extends Command {
  private SwerveDrive swerveDrive;
  private Double angle, turn;
  private PIDController pid;
  private ChassisSpeeds chassisSpeeds;
  private SlewRateLimiter tLimiter;
  private Boolean done;

  /** Creates a new AutoTurnToHeading. */
  public AutoTurnToHeading(SwerveDrive swerveDrive, Double angle) {
    this.swerveDrive = swerveDrive;
    this.angle = angle;
    this.done = false;
    this.tLimiter = new SlewRateLimiter(DriveConstants.MaxAngularAccelerationUnitsPerSecond);
    pid = new PIDController(SwerveConstants.PIDProportional, 0, 0.01);
    pid.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = tLimiter.calculate(pid.calculate(-swerveDrive.getHeading(), angle));
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turn,
        swerveDrive.getRotation2d());
    swerveDrive.drive(chassisSpeeds);

    done = (turn < 0.1);
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
    return done;
  }
}