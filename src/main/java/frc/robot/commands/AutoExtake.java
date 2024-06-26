// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.LauncherMech;

public class AutoExtake extends Command {
private LauncherMech launcherMech;

  /** Creates a new SetAutoLaunchSpeed. */
  public AutoExtake(LauncherMech launcherMech) {
    this.launcherMech = launcherMech;
    addRequirements(launcherMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(launcherMech.speakerReady()){
      launcherMech.setFeedSpeed(LauncherConstants.feedSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherMech.setFeedSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return launcherMech.getProxSensorValue();
  }
}