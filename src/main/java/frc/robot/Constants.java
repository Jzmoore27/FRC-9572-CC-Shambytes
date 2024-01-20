// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public final class Constants {
  public static final class SwerveConstants { // motor0 Turning, motor1 Drive
    // Drive Wheel Diameter
    public static final double driveWheelDiameterMeters = 0.1;
    // Front Left
    public static final BaseTalon flMotor0 = new BaseTalon(1, "");
    public static final BaseTalon flMotor1 = new BaseTalon(1, "");
    public static final AnalogInput flAbsEncoder = new AnalogInput(0);
    // Front Right
    public static final BaseTalon frMotor0 = new BaseTalon(1, "");
    public static final BaseTalon frMotor1 = new BaseTalon(1, "");
    public static final AnalogInput frAbsEncoder = new AnalogInput(0);
    // Back Left
    public static final BaseTalon blMotor0 = new BaseTalon(1, "");
    public static final BaseTalon blMotor1 = new BaseTalon(1, "");
    public static final AnalogInput blAbsEncoder = new AnalogInput(0);
    // Back Right
    public static final BaseTalon brMotor0 = new BaseTalon(1, "");
    public static final BaseTalon brMotor1 = new BaseTalon(1, "");
    public static final AnalogInput brAbsEncoder = new AnalogInput(0);

    // PID
    public static final double PIDProportional = 0.5;
  }

  public static final class DriveConstants {
    public static final double MaxMPS = 1;
    public static final double MaxAccelerationUnitsPerSecond = 3;
    public static final double MaxAngularAccelerationUnitsPerSecond = 3;
    public static final double PhysicalMaxAngularSpeedRPS = 2 * 2 * Math.PI;
    public static final double MaxAngularSpeedRPS = PhysicalMaxAngularSpeedRPS / 4;

    public static final double WheelBase = Units.inchesToMeters(25.5);
    public static final double TrackWidth = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WheelBase / 2, -TrackWidth / 2),
        new Translation2d(WheelBase / 2, TrackWidth / 2),
        new Translation2d(-WheelBase / 2, -TrackWidth / 2),
        new Translation2d(-WheelBase / 2, TrackWidth / 2));
  }

  public static final class OIConstants {
    public static final int DriverControllerPort = 0;

    public static final int DriverXAxis = 0;
    public static final int DriverYAxis = 1;
    public static final int DriverRotAxis = 2;
    public static final int DriverFieldOrientedButtonIdx = 1;

    public static final double Deadband = 0.05;
  }
}