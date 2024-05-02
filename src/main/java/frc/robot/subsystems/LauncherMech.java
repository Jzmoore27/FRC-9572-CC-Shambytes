// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LauncherMech extends SubsystemBase {

  private TalonFX launchMotor1, launchMotor2, feedMotor, armMotor2;
  private DutyCycleEncoder absArm;
  private DigitalInput proxSensor = new DigitalInput(1);
  public LauncherMech(TalonFX launchMotor1, TalonFX launchMotor2, TalonFX feedMotor, TalonFX armMotor2) {
    absArm = new DutyCycleEncoder(7);
    feedMotor.setInverted(true);
    launchMotor1.setInverted(true);
    launchMotor2.setInverted(true);
    //armMotor.setNeutralMode(NeutralModeValue.Coast);
    armMotor2.setNeutralMode(NeutralModeValue.Coast);
    armMotor2.setInverted(true);
    this.launchMotor1 = launchMotor1;
    this.launchMotor2 = launchMotor2;
    this.feedMotor = feedMotor;
    //this.armMotor = armMotor;
    this.armMotor2 = armMotor2;
  }

  // Note Handling
  public void setLaunchSpeed(Double LowerSpeed, Double UpperSpeed) {
    launchMotor1.set(LowerSpeed);
    launchMotor2.set(UpperSpeed);
  }

  public void setFeedSpeed(Double speed) {
    feedMotor.set(speed);
  }

  public void stopFeed(){
    feedMotor.stopMotor();
  }

  public void stopMotors() {
    launchMotor1.stopMotor();
    launchMotor2.stopMotor();
    feedMotor.stopMotor();
  }

  // Arm Movement
  public void stopArm() {
    //armMotor.stopMotor();
    armMotor2.stopMotor();
  }

  public void zeroArm() {
    //armMotor.setPosition(0);
    armMotor2.setPosition(0);
  }

  public Boolean speakerReady(){
    return launchMotor1.getVelocity().getValue() > 65.0;
  }

  public Boolean ampReady(){
    return launchMotor1.getVelocity().getValue() > 22.0;
  }

  public Boolean launchRunning(){
    return launchMotor1.getVelocity().getValue() > 0.05;
  }

  public Boolean hanging(){
    return launchMotor1.getVelocity().getValue() > 1.0;
  }

  public Boolean harmony(){
    return launchMotor1.getVelocity().getValue() > 1.0;
  }

  public Boolean intake(){
    return launchMotor1.getVelocity().getValue() < -3;
  }

  public Boolean getProxSensorValue() {
    return proxSensor.get();
  }

  public double getAbsPositionDeg() {
    return (absArm.get() * 360) - 181;
  }

  public void setArmSpeed(Double speed) {
    //armMotor.set(speed / 18);
    armMotor2.set(speed / 18);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("absAngle", getAbsPositionDeg());
    SmartDashboard.putBoolean("proxSensor", getProxSensorValue());
    SmartDashboard.putNumber("launch speed", launchMotor1.getVelocity().getValue());
  }
}