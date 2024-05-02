// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherMech;
import frc.robot.Constants.*;

public class TeleLauncherCmd extends Command {
  LauncherMech launcherMech;
  Supplier<Double> launcherSpeedSupplier, rawArmSupplier;
  Supplier<Boolean> feedOnSupplier, reversedSupplier, height1Supplier, height2Supplier, speedToggle, ampControl, trapControl;
  Double maxLauncherSpeed1, maxLauncherSpeed2, launcherSpeed1, launcherSpeed2, feedSpeed, reverseMultiplier = 1.0, rawArm;
  Double targetAngle = 1.0;
  Boolean feedOn, reversed, pressed;
  SlewRateLimiter feedLimiter = new SlewRateLimiter(LauncherConstants.feedLimitRate);
  PIDController armDownPID = new PIDController(0.4, 0, 0.01);
  PIDController armUpPID = new PIDController(0.4, 0.1, 0.02);

  /** Creates a new TeleLauncherCmd. */
  public TeleLauncherCmd(LauncherMech launcherMech, Supplier<Double> launcherSpeed, Supplier<Boolean> feedOn,
      Supplier<Boolean> reversed, Supplier<Boolean> height1, Supplier<Boolean> height2, Supplier<Boolean> speedToggle, Supplier<Boolean> ampControl, Supplier<Double> rawArm) {
    this.launcherMech = launcherMech;
    this.targetAngle = 1.0;
    this.launcherSpeedSupplier = launcherSpeed;
    this.feedOnSupplier = feedOn;
    this.reversedSupplier = reversed;
    this.height1Supplier = height1;
    this.height2Supplier = height2;
    this.speedToggle = speedToggle;
    this.ampControl = ampControl;
    this.rawArmSupplier = rawArm;
    this.maxLauncherSpeed1 = LauncherConstants.maxLauncherSpeed;
    this.maxLauncherSpeed2 = LauncherConstants.maxLauncherSpeed;
    this.pressed = false;

    addRequirements(launcherMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherMech.stopMotors();
    launcherMech.stopArm();
    launcherMech.zeroArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // NOTE CODE

    // Get supplied data
    reversed = reversedSupplier.get();
    launcherSpeed1 = launcherSpeedSupplier.get();
    launcherSpeed2 = launcherSpeedSupplier.get();
    feedOn = feedOnSupplier.get();
    reverseMultiplier = reversed ? -1.0 : 1.0;
    // calculate feed motor speed
    feedSpeed = feedOn ? LauncherConstants.feedSpeed * reverseMultiplier : 0;
    //feedSpeed = !(launcherSpeed > 0)&&!launcherMech.getProxSensorValue() ? 0.0 : feedSpeed;
    // apply reverse speeds
    // set launcher wheel speed
    if(ampControl.get()){
     maxLauncherSpeed1 = LauncherConstants.maxAmpSpeed;
     maxLauncherSpeed2 = LauncherConstants.maxAmpSpeed-0.08;
    }
    else {
      maxLauncherSpeed1 = LauncherConstants.maxLauncherSpeed;
      maxLauncherSpeed2 = LauncherConstants.maxLauncherSpeed;
    }
    if (launcherSpeed1 > 0) {
      launcherSpeed1 = (launcherSpeed1 > maxLauncherSpeed1) ? maxLauncherSpeed1
          : launcherSpeed1;
      launcherSpeed2 = (launcherSpeed2 > maxLauncherSpeed2) ? maxLauncherSpeed2
          : launcherSpeed2;
      launcherSpeed1 = launcherSpeed1 * reverseMultiplier;
      launcherSpeed1 = reversed ? launcherSpeed1 * LauncherConstants.reversedLauncherMultiplier : launcherSpeed1;
      launcherSpeed2 = launcherSpeed2 * reverseMultiplier;
      launcherSpeed2 = reversed ? launcherSpeed2 * LauncherConstants.reversedLauncherMultiplier : launcherSpeed2;
      launcherMech.setLaunchSpeed(launcherSpeed1, launcherSpeed2);
    }

    // stop launch motor
    else {
      launcherMech.setLaunchSpeed(0.0, 0.0);
    }
    // Set feed motor speed
    launcherMech.setFeedSpeed(feedSpeed);

    // END OF NOTE CODE / START OF ARM CODE

    SmartDashboard.putNumber("armdiff", Math.abs(launcherMech.getAbsPositionDeg()-targetAngle));
    SmartDashboard.putNumber("targetAngle", targetAngle);
    
    if(height1Supplier.get()){
      targetAngle = 1.75;
    }

    if(height2Supplier.get()){
      targetAngle = 70.5;
    }

    if((targetAngle-launcherMech.getAbsPositionDeg()>2)){
      launcherMech.setArmSpeed(armDownPID.calculate(launcherMech.getAbsPositionDeg(),targetAngle));}
    else if((targetAngle-launcherMech.getAbsPositionDeg()<-2)){
      launcherMech.setArmSpeed(armUpPID.calculate(launcherMech.getAbsPositionDeg(),targetAngle));}
    else{
      launcherMech.stopArm();
    }
    if(speedToggle.get()){
      if(!pressed) {
        maxLauncherSpeed1 = maxLauncherSpeed1 == 0.7 ? 0.3: 0.7;
        maxLauncherSpeed2 = maxLauncherSpeed2 == 0.7 ? 0.3: 0.7;
        pressed = true;
      }
    }
    else
    {pressed = false;}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherMech.stopMotors();
    launcherMech.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
