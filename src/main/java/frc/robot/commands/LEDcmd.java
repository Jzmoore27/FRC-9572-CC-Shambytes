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
  Integer amt, i = 0, blink = 255;
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
    candle.setLeds(0,amt,0,0,255);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i++;
    if(i >= 9){
       blink = blink==255 ? 0 : 255;
       i = 0;
    }
    if(launcherMech.speakerReady()){
      candle.setLeds(0,amt,255,0,0);
    }
    /*else if(launcherMech.ampReady()){
      candle.setLeds(0,amt,255,255,0);
    }*/
    else if(launcherMech.launchRunning()){
      candle.setLeds(0,amt,0,blink,0);
    }
    else if(launcherMech.intake()){
      candle.setLeds(0,amt,blink-152,0,blink-105);
    }
    else if((!launcherMech.getProxSensorValue())&&launcherMech.getAbsPositionDeg()<20){
      candle.setLeds(0,amt,160,32,40);
    }
    else{
      candle.setLeds(0,amt,0,0,blink-155);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candle.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
