// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants.*;
import frc.robot.subsystems.SwerveDrive;

public class SwerveJoystickCmd extends Command {

  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction, zeroButton;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private  Boolean fieldRelative, fPressed, zPressed = true;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveDrive swerveDrive,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> zeroFunction) {
    this.swerveDrive = swerveDrive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.zeroButton = zeroFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.MaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.MaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fieldRelative = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    if(!fieldOrientedFunction.get()){
      if(!fPressed) {
        fieldRelative = fieldRelative ? false: true;
        fPressed = true;
      }
    }
    else{fPressed = false;}

    if(zeroButton.get()){
      if(!zPressed) {
        swerveDrive.zeroHeading();
        zPressed = true;
      }
    }
    else{zPressed = false;}

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > IOConstants.Deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.Deadband ? ySpeed : 0.0;

    xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed : 0.0;
    
    turningSpeed = Math.abs(turningSpeed) > IOConstants.Deadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.MaxMPS;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.MaxMPS;
    turningSpeed = turningLimiter.calculate(turningSpeed)*-1
        * DriveConstants.MaxAngularSpeedRPS;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDrive.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveDrive.setDesiredStates(moduleStates);

    SmartDashboard.putBoolean("fieldOriented", fieldRelative);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
     SmartDashboard.putNumber("turnSpeed", turningSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
