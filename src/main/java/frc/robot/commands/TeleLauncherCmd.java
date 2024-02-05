// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherMech;
import frc.robot.Constants.*;

public class TeleLauncherCmd extends Command {
  LauncherMech shooterMech;
  Supplier<Double> launcherSpeedSupplier;
  Supplier<Boolean> feedOnSupplier, reversedSupplier;
  Double launcherSpeed, feedSpeed, reverseMultiplier;
  Boolean feedOn, reversed;
  SlewRateLimiter launchLimiter = new SlewRateLimiter(LauncherConstants.launcherLimitRate);
  SlewRateLimiter feedLimiter = new SlewRateLimiter(LauncherConstants.feedLimitRate);
  
  /** Creates a new TeleLauncherCmd. */
  public TeleLauncherCmd(LauncherMech shooterMech, Supplier<Double> launcherSpeed, Supplier<Boolean> feedOn, Supplier<Boolean> reversed) {
    this.shooterMech = shooterMech;
    this.launcherSpeedSupplier = launcherSpeed;
    this.feedOnSupplier = feedOn;
    this.reversedSupplier = reversed;

    addRequirements(shooterMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterMech.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get supplied data
    reversed = reversedSupplier.get();
    launcherSpeed = launcherSpeedSupplier.get();
    feedOn = feedOnSupplier.get();
    //calculate feed motor speed
    feedSpeed = feedOn ? LauncherConstants.feedSpeed * reverseMultiplier : 0;
    feedSpeed = launchLimiter.calculate(feedSpeed);
    //apply reverse speeds
    reverseMultiplier = reversed ? -1.0 : 1.0;
    //set launcher wheel speed
    if(launcherSpeed > 0.1) {
      launcherSpeed = (launcherSpeed > LauncherConstants.maxLauncherSpeed) ? LauncherConstants.maxLauncherSpeed : launcherSpeed;
      launcherSpeed = launcherSpeed * reverseMultiplier;
      launcherSpeed = reversed ? launcherSpeed * LauncherConstants.reversedLauncherMultiplier : launcherSpeed;
      launcherSpeed = launchLimiter.calculate(launcherSpeed);
      shooterMech.setLaunchSpeed(launcherSpeed);
    }
    //Stop launch motor
    else{shooterMech.setLaunchSpeed(0);}
    //Set feed motor speed
    shooterMech.setFeedSpeed(feedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMech.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
