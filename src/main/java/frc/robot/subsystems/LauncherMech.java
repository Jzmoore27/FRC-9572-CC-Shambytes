// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix6.hardware.TalonFX;

public class LauncherMech extends SubsystemBase {

  private TalonFX launchMotor1, launchMotor2, feedMotor, armMotor;

  public LauncherMech(TalonFX launchMotor1, TalonFX launchMotor2, TalonFX feedMotor, TalonFX armMotor) {
    this.launchMotor1 = launchMotor1;
    this.launchMotor2 = launchMotor2;
    this.feedMotor = feedMotor;
    this.armMotor = armMotor;
  }

  //Note Handling
  public void setLaunchSpeed(double speed) {
    launchMotor1.set(speed);
    launchMotor2.set(speed);
  }

  public void setFeedSpeed(double speed) {
    feedMotor.set(speed);
  }

  public void stopMotors() {
    launchMotor1.set(0);
    launchMotor2.set(0);
    feedMotor.set(0);
  }

  //Arm Movement
  public void stopArm() {
    armMotor.set(0);
  }

  public void zeroArm() {
    armMotor.setPosition(0);
  }

  public double getArmPositionDeg() {
    return(armMotor.getPosition().getValue() * 360 * LauncherConstants.armMotorRatio);
  }

  public void setArmSpeed(Double speed) {
    armMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}