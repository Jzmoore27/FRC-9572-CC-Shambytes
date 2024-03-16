// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public final class Constants {
  public static final class SwerveDriveConstants {
    public static final class SwerveConstants { // motor0 Turning, motor1 Drive
      // Drive Wheel Diameter
      public static final double driveWheelCircumferenceMeters = 0.3141;
      // Front Left
      public static final TalonFX flMotor0 = new TalonFX(3);
      public static final TalonFX flMotor1 = new TalonFX(2);
      public static final CANcoder flAbsEncoder = new CANcoder(1);
      // Front Right
      public static final TalonFX frMotor0 = new TalonFX(1);
      public static final TalonFX frMotor1 = new TalonFX(0);
      public static final CANcoder frAbsEncoder = new CANcoder(0);
      // Back Left
      public static final TalonFX blMotor0 = new TalonFX(5);
      public static final TalonFX blMotor1 = new TalonFX(4);
      public static final CANcoder blAbsEncoder = new CANcoder(2);
      // Back Right
      public static final TalonFX brMotor0 = new TalonFX(7);
      public static final TalonFX brMotor1 = new TalonFX(6);
      public static final CANcoder brAbsEncoder = new CANcoder(3);

      // PID
      public static final double PIDProportional = 0.2;
    }

    public static final class DriveConstants {
      public static final double MaxMPS = 1;
      public static final double MaxAccelerationUnitsPerSecond = 1;
      public static final double MaxAngularAccelerationUnitsPerSecond = 1.5;
      public static final double PhysicalMaxAngularSpeedRPS = 4 * Math.PI;
      public static final double MaxAngularSpeedRPS = PhysicalMaxAngularSpeedRPS / 4;

      public static final double WheelBase = Units.inchesToMeters(25);
      public static final double TrackWidth = Units.inchesToMeters(5);

      public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
          new Translation2d(WheelBase / 2, -TrackWidth / 2),
          new Translation2d(WheelBase / 2, TrackWidth / 2),
          new Translation2d(-WheelBase / 2, -TrackWidth / 2),
          new Translation2d(-WheelBase / 2, TrackWidth / 2));
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = DriveConstants.MaxMPS / 4;
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.MaxMPS / 10;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double PXController = 1.5;
        public static final double PYController = 1.5;
        public static final double PThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        MaxAngularSpeedRadiansPerSecond,
                        MaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class IOConstants {
      public static final int DriverControllerPort = 0;
      public static final int DriverController2Port = 1;

      public static final int DriverXAxis = 0;
      public static final int DriverYAxis = 1;
      public static final int DriverRotAxis = 0;
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
    public static final TalonFX armMotor = new TalonFX(8);
    // Motor Ratio
    public static final double armMotorRatio = 0.002747252747;
    // Speeds
    public static final double maxLauncherSpeed = 0.7;
    public static final double maxAmpSpeed = 0.2;
    public static final double feedSpeed = 0.2;
    public static final double reversedLauncherMultiplier = 0.6;
    // Limiters
    public static final double feedLimitRate = 0.2;
    // Controller
    public static final int CoDriverControllerPort = 2;
    // Buttons
    public static final int launchSpeedAxis = 3; // Right Trigger
    public static final int rawArmAxis = 5; // Right Joystick Up/Down
    public static final int feedButtonIdx = 6; // Right Button
    public static final int reverseButtonIdx = 5; // Left Button
    public static final int ampButtonIdx = 3; // Y Button
    public static final int height2ButtonIdx = 2; // B Button
    public static final int height1ButtonIdx = 1; // A Button 
    public static final int launchSpeedButtonIdx = 3; // X Button 
  }
}