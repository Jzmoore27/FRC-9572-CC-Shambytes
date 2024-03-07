// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoLaunchSpeed;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TeleLauncherCmd;
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
  private final SendableChooser<Command> autoChooser;

  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final LauncherMech shooterMech = new LauncherMech(LauncherConstants.launchMotor1,
      LauncherConstants.launchMotor2, LauncherConstants.feedMotor, LauncherConstants.armMotor);
  private final Joystick driverJoystick = new Joystick(IOConstants.DriverControllerPort);
  private final Joystick coDriverJoystick = new Joystick(LauncherConstants.CoDriverControllerPort);
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("AutoIntake", new AutoIntake(shooterMech));
    NamedCommands.registerCommand("startArm", new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed));
    NamedCommands.registerCommand("stopArm", new AutoLaunchSpeed(shooterMech, 0.0));
 
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive,
        () -> driverJoystick.getRawAxis(IOConstants.DriverYAxis),
        () -> driverJoystick.getRawAxis(IOConstants.DriverXAxis),
        () -> -driverJoystick.getRawAxis(IOConstants.DriverRotAxis),
        () -> !driverJoystick.getRawButton(IOConstants.DriverFieldOrientedButtonIdx),
        () -> !driverJoystick.getRawButton(IOConstants.zeroButtonIdx)));

    shooterMech.setDefaultCommand(new TeleLauncherCmd(shooterMech,
        () -> coDriverJoystick.getRawAxis(LauncherConstants.launchSpeedAxis),
        () -> coDriverJoystick.getRawButton(LauncherConstants.feedButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.reverseButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.height1ButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.height2ButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.ampButtonIdx),
        () -> coDriverJoystick.getRawButton(LauncherConstants.launchSpeedButtonIdx),
        () -> coDriverJoystick.getRawAxis(LauncherConstants.rawArmAxis)));
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

    //return new PathPlannerAuto("Auto");

    return new SequentialCommandGroup(
        new AutoLaunchSpeed(shooterMech, LauncherConstants.maxLauncherSpeed),
        new AutoSleep(2.0),
        new AutoIntake(shooterMech),
        new AutoSleep(0.4),
        new AutoLaunchSpeed(shooterMech, 0.0)
        );
  }
}