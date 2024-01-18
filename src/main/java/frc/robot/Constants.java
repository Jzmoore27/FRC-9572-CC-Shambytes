// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public final class Constants {
  public static final class SwerveConstants { // motor0 Turning, motor1 Drive
    // Drive Wheel Diameter
    public static final double driveWheelDiameterMeters = 0.1;
    // Front Left
    public static final BaseTalon flMotor0 = new BaseTalon(1,"");
    public static final BaseTalon flMotor1 = new BaseTalon(1,""); 
    public static final AnalogInput flAbsEncoder = new AnalogInput(0);
    // Front Right
    public static final BaseTalon frMotor0 = new BaseTalon(1,""); 
    public static final BaseTalon frMotor1 = new BaseTalon(1,""); 
    public static final AnalogInput frAbsEncoder = new AnalogInput(0);
    // Back Left
    public static final BaseTalon blMotor0 = new BaseTalon(1,""); 
    public static final BaseTalon blMotor1 = new BaseTalon(1,""); 
    public static final AnalogInput blAbsEncoder = new AnalogInput(0);
    // Back Right
    public static final BaseTalon brMotor0 = new BaseTalon(1,""); 
    public static final BaseTalon brMotor1 = new BaseTalon(1,""); 
    public static final AnalogInput brAbsEncoder = new AnalogInput(0);

    // PID
    public static final double PIDProportional = 0.5;
  }

  public static final class DriveConstants {
    public static final double MaxMPS = 1;
  }
}