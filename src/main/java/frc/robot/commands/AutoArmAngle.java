// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherMech;

public class AutoArmAngle extends Command {
  LauncherMech launcherMech;
  PIDController armPID = new PIDController(0.3, 0, 0.01);
  Double targetAngle, angleDiff = 3.0;
  /** Creates a new AutoArmAngle. */
  public AutoArmAngle(LauncherMech launcherMech, Double targetAngle) {
    this.launcherMech = launcherMech;
    this.targetAngle = targetAngle;
    addRequirements(launcherMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherMech.stopArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleDiff = Math.abs(launcherMech.getAbsPositionDeg()-targetAngle);
    launcherMech.setArmSpeed(armPID.calculate(launcherMech.getAbsPositionDeg(),targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherMech.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleDiff < 3;
  }
}
