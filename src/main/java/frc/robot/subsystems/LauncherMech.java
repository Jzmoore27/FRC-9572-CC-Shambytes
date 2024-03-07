// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LauncherMech extends SubsystemBase {

  private TalonFX launchMotor1, launchMotor2, feedMotor, armMotor;
  private DutyCycleEncoder absArm;
  private DigitalInput proxSensor = new DigitalInput(1);
  public LauncherMech(TalonFX launchMotor1, TalonFX launchMotor2, TalonFX feedMotor, TalonFX armMotor) {
    absArm = new DutyCycleEncoder(0);
    feedMotor.setInverted(true);
    launchMotor1.setInverted(true);
    launchMotor2.setInverted(true);
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    this.launchMotor1 = launchMotor1;
    this.launchMotor2 = launchMotor2;
    this.feedMotor = feedMotor;
    this.armMotor = armMotor;
  }

  // Note Handling
  public void setLaunchSpeed(Double speed) {
    launchMotor1.set(speed);
    launchMotor2.set(speed);
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
    armMotor.stopMotor();
  }

  public void zeroArm() {
    armMotor.setPosition(0);
  }

  public Boolean getProxSensorValue() {
    return proxSensor.get();
  }

  public double getAbsPositionDeg() {
    return (absArm.get() * 360) - 11.2;
  }

  public void setArmSpeed(Double speed) {
    armMotor.set(speed / 20);
  }

  public void resetArm(){
    //armMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}