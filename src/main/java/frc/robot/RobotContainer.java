// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.SwerveDriveConstants.IOConstants;
import frc.robot.commands.AutoArmAngle;
import frc.robot.commands.AutoDistanceDrive;
import frc.robot.commands.AutoDriveTimed;
import frc.robot.commands.AutoExtake;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoSleep;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.LEDcmd;
import frc.robot.commands.AutoLaunchSpeed;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TeleLauncherCmd;
import frc.robot.subsystems.CANdleLED;
import frc.robot.subsystems.LauncherMech;
import frc.robot.subsystems.SwerveDrive;

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

  private final SwerveDrive swerveDrive;
  private Double mult = 1.0;
  private final CANdle ctreCandle = new CANdle(0);
  private final CANdleLED candle = new CANdleLED(ctreCandle);
  private final LauncherMech shooterMech = new LauncherMech(LauncherConstants.launchMotor1,
      LauncherConstants.launchMotor2, LauncherConstants.feedMotor, LauncherConstants.armMotor);
  private final Joystick driverJoystick = new Joystick(IOConstants.DriverControllerPort);
  private final Joystick driverJoystick2 = new Joystick(IOConstants.DriverController2Port);
  private final Joystick coDriverJoystick = new Joystick(LauncherConstants.CoDriverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Auton Commands
  private final Command stn, sonl, aonl, ctnus;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = new SwerveDrive();

    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive,
        () -> driverJoystick.getRawAxis(IOConstants.DriverYAxis) * 1.2,
        () -> driverJoystick.getRawAxis(IOConstants.DriverXAxis) * 1.2,
        () -> -driverJoystick2.getRawAxis(IOConstants.DriverRotAxis),
        () -> !driverJoystick2.getRawButton(IOConstants.DriverFieldOrientedButtonIdx),
        () -> !driverJoystick.getRawButton(IOConstants.zeroButtonIdx)));

    shooterMech.setDefaultCommand(new TeleLauncherCmd(shooterMech,
        () -> coDriverJoystick.getRawAxis(LauncherConstants.launchSpeedAxis),
        () -> coDriverJoystick.getRawButton(LauncherConstants.feedButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.reverseButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.height1ButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.height2ButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.ampButtonIdx),
        //() -> coDriverJoystick.getRawButton(LauncherConstants.trapButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.launchSpeedButtonIdx),
        () -> coDriverJoystick.getRawAxis(LauncherConstants.rawArmAxis)));

    candle.setDefaultCommand(new LEDcmd(candle, shooterMech, 60));
    // Configure the trigger bindings
    configureBindings();

    // Initialize Auto Commands
    stn = new SequentialCommandGroup(
        new AutoArmAngle(shooterMech, 64.0),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(2.0),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new ParallelCommandGroup(
          new AutoDistanceDrive(swerveDrive, 1.5, -0.21, -0.19*mult, 0.2*mult),
          new AutoIntake(shooterMech),
          new SequentialCommandGroup(
            new AutoArmAngle(shooterMech, 0.0))),
        new PrintCommand("made it"),
        new AutoDistanceDrive(swerveDrive, 0.63, 0.13, -0.1*mult, -0.13*mult),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(0.7),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new AutoDistanceDrive(swerveDrive, 2.0, 0.03, 0.4*mult, 0.13*mult)
    );

    sonl = new SequentialCommandGroup(
        new AutoArmAngle(shooterMech, 64.0),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(2.0),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new AutoDistanceDrive(swerveDrive, 4.13, -0.4, 0.0*mult, 0.0*mult)
    );

    aonl = new SequentialCommandGroup(
        new AutoArmAngle(shooterMech, 64.0),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(2.0),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new AutoDistanceDrive(swerveDrive, 1.5, -0.2, 0.0*mult, -0.093),
        new AutoSleep(3.0),
        new AutoDistanceDrive(swerveDrive, 4.4, -0.35, -0.02, -0.0*mult)
    );
    ctnus = new SequentialCommandGroup(
        new AutoArmAngle(shooterMech, 64.0),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(2.0),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new AutoArmAngle(shooterMech, 0.0),
        new ParallelCommandGroup(
          new AutoIntake(shooterMech),
          new AutoDistanceDrive(swerveDrive, 1.2, -0.2, -0.0*mult, 0.0*mult)),
        new AutoDistanceDrive(swerveDrive, 0.6, 0.2, 0.0*mult, 0.0*mult),
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(0.7),
        new AutoExtake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0),
        new AutoDistanceDrive(swerveDrive, 3.0, -0.2, 0.13*mult, 0.0*mult)
    );

    // Set up autoChooser
    autoChooser.setDefaultOption("Choose an Auto", null);
    autoChooser.addOption("Source 2Note", stn);
    autoChooser.addOption("Source 1Note + leave", sonl);
    autoChooser.addOption("Amp 1Note + leave", aonl);
    autoChooser.addOption("Center 2note under stage", ctnus);

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            mult = -1.0;
        }
        if (ally.get() == Alliance.Blue) {
            mult = 1.0;
        }
    }
    else {
        mult = 1.0;
    }
    return autoChooser.getSelected();
  }
}