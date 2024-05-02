// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix.led.CANdle;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.SwerveDriveConstants.IOConstants;
import frc.robot.commands.AutoArmAngle;
import frc.robot.commands.AutoExtake;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoLaunchSpeed;
import frc.robot.commands.AutoSleep;
import frc.robot.commands.LEDcmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TeleLauncherCmd;
import frc.robot.subsystems.CANdleLED;
import frc.robot.subsystems.LauncherMech;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveDriveSub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final LimeLight limelight = new LimeLight();
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> speedMult = new SendableChooser<Double>();
  private final SwerveDriveSub swerveDrive = new SwerveDriveSub(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CANdle ctreCandle = new CANdle(0);
  private final CANdleLED candle = new CANdleLED(ctreCandle);
  private final LauncherMech launcherMech = new LauncherMech(LauncherConstants.launchMotor1,
      LauncherConstants.launchMotor2, LauncherConstants.feedMotor, LauncherConstants.armMotor2);
  private final Joystick driverJoystick = new Joystick(IOConstants.DriverControllerPort);
  // --I!-- private final Joystick driverJoystick2 = new Joystick(IOConstants.DriverController2Port);
  private final Joystick coDriverJoystick = new Joystick(LauncherConstants.CoDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("preload", new SequentialCommandGroup(
      new AutoLaunchSpeed(launcherMech, 0.8),
      new AutoSleep(0.7),
      new AutoIntake(launcherMech),
      new AutoExtake(launcherMech),
      new AutoLaunchSpeed(launcherMech, 0.0)
    ));
    NamedCommands.registerCommand("intake", new AutoIntake(launcherMech));
    NamedCommands.registerCommand("LowArm", new AutoArmAngle(launcherMech, 3.5));
    NamedCommands.registerCommand("HighArm", new AutoArmAngle(launcherMech, 74.5));
    NamedCommands.registerCommand("CenterZero", new InstantCommand(()->{swerveDrive.setYaw(swerveDrive.getHeading().getDegrees()-180);}));
    autoChooser = AutoBuilder.buildAutoChooser();
    speedMult.addOption("Full Speed", 1.2);
    speedMult.addOption("Half Speed", 0.6);
    speedMult.addOption("Crawl", 0.2);
    SmartDashboard.putData("Speed Chooser", speedMult);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(
      swerveDrive,
      limelight,
      () -> driverJoystick.getRawAxis(IOConstants.DriverYAxis) * speedMult.getSelected(),
      () -> driverJoystick.getRawAxis(IOConstants.DriverXAxis) * speedMult.getSelected(),
      () -> -driverJoystick.getRawAxis(IOConstants.DriverRotAxis) * speedMult.getSelected(),
      () -> driverJoystick.getRawButton(IOConstants.DriverFieldOrientedButtonIdx),
      () -> driverJoystick.getRawButton(IOConstants.zeroButtonIdx),
      () -> driverJoystick.getRawButton(1)
    ));

    launcherMech.setDefaultCommand(new TeleLauncherCmd(
      launcherMech,
      () -> coDriverJoystick.getRawAxis(LauncherConstants.launchSpeedAxis),
      () -> coDriverJoystick.getRawButton(LauncherConstants.feedButtonIdx),
      () -> coDriverJoystick.getRawButton(LauncherConstants.reverseButtonIdx),
      () -> coDriverJoystick.getRawButton(LauncherConstants.height1ButtonIdx),
      () -> coDriverJoystick.getRawButton(LauncherConstants.height2ButtonIdx),
      () -> coDriverJoystick.getRawButton(LauncherConstants.ampButtonIdx),
      () -> coDriverJoystick.getRawButton(LauncherConstants.launchSpeedButtonIdx),
      () -> coDriverJoystick.getRawAxis(LauncherConstants.rawArmAxis)
      ));

    candle.setDefaultCommand(new LEDcmd(candle, launcherMech, 60));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}