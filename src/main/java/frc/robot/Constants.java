// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public final class Constants {
  public static final class SwerveConstants { // motor0 Turning, motor1 Drive
    // Drive Wheel Diameter
    public static final double driveWheelDiameterMeters = 0.1;
    // Front Left
    public static final PWMTalonFX flMotor0 = new PWMTalonFX(0);
    public static final PWMTalonFX flMotor1 = new PWMTalonFX(0);
    public static final AnalogInput flAbsEncoder = new AnalogInput(0);
    // Front Right
    public static final PWMTalonFX frMotor0 = new PWMTalonFX(0);
    public static final PWMTalonFX frMotor1 = new PWMTalonFX(0);
    public static final AnalogInput frAbsEncoder = new AnalogInput(0);
    // Back Left
    public static final PWMTalonFX blMotor0 = new PWMTalonFX(0);
    public static final PWMTalonFX blMotor1 = new PWMTalonFX(0);
    public static final AnalogInput blAbsEncoder = new AnalogInput(0);
    // Back Right
    public static final PWMTalonFX brMotor0 = new PWMTalonFX(0);
    public static final PWMTalonFX brMotor1 = new PWMTalonFX(0);
    public static final AnalogInput brAbsEncoder = new AnalogInput(0);

    // PID
    public static final double PIDProportional = 0.5;
  }

  public static final class DriveConstants {
    public static final double MaxMPS = 1;
  }
}