// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLED;
import frc.robot.subsystems.LauncherMech;

public class LEDcmd extends Command {
  CANdleLED candle;
  LauncherMech launcherMech;
  Integer amt, i;
  /** Creates a new LEDcmd. */
  public LEDcmd(CANdleLED candle, LauncherMech launcherMech, Integer amt) {
    this.candle = candle;
    this.launcherMech = launcherMech;
    this.amt = amt;
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candle.setLeds(0,amt+7,0,0,255);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!launcherMech.getProxSensorValue()){
      candle.setLeds(0, amt/10, 220, 120, 10);
      candle.setLeds((amt/10)*3, (amt/10)*4, 220, 120, 10);
      candle.setLeds((amt/10)*6, (amt/10)*7, 220, 120, 10);
      candle.setLeds((amt/10)*9, amt, 220, 120, 10);
    }
    if(launcherMech.getProxSensorValue()){
      candle.setLeds(0, amt/10, 0, 0, 0);
      candle.setLeds((amt/10)*3, (amt/10)*4, 0, 0, 0);
      candle.setLeds((amt/10)*6, (amt/10)*7, 0, 0, 0);
      candle.setLeds((amt/10)*9, amt, 0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
