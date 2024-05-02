// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.PIDConstants;


public final class Constants {

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.1, 0.2, 0.01);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.6, 0, 0.01);
  }

  public static final class SwerveDriveConstants {
    public static final class IOConstants {
      public static final int DriverControllerPort = 0;
      public static final int DriverController2Port = 1;

      public static final int DriverXAxis = 0;
      public static final int DriverYAxis = 1;
      public static final int DriverRotAxis = 4;
      public static final int DriverFieldOrientedButtonIdx = 4;
      public static final int zeroButtonIdx = 3;

      public static final double Deadband = 0.05;
    }
  }

  public static final class LauncherConstants {
    // Motors -- in order from first to last on CAN bus
    public static final TalonFX launchMotor1 = new TalonFX(10);
    public static final TalonFX launchMotor2 = new TalonFX(11);
    public static final TalonFX feedMotor = new TalonFX(9);
    //public static final TalonFX armMotor = new TalonFX(8);
    public static final TalonFX armMotor2 = new TalonFX(12);
    // Motor Ratio
    public static final double armMotorRatio = 0.002747252747;
    // Speeds
    public static final double maxLauncherSpeed = 0.95;
    public static final double maxAmpSpeed = 0.3;
    public static final double feedSpeed = 0.3;
    public static final double reversedLauncherMultiplier = 0.4;
    // Limiters
    public static final double feedLimitRate = 0.2;
    // Controller
    public static final int CoDriverControllerPort = 2;
    // Buttons
    public static final int launchSpeedAxis = 3; // Right Trigger
    public static final int rawArmAxis = 5; // Right Joystick Up/Down
    public static final int feedButtonIdx = 6; // Right Button
    public static final int reverseButtonIdx = 5; // Left Button
    public static final int ampButtonIdx = 3; // X Button
    public static final int height2ButtonIdx = 2; // B Button
    public static final int height1ButtonIdx = 1; // A Button 
    public static final int launchSpeedButtonIdx = 3; // X Button 
    public static final int trapButtonIdx = 4; // Y Button 
  }
}