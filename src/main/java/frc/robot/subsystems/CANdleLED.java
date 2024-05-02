// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleLED extends SubsystemBase {
  /** Creates a new CANdleLED. */
  private CANdle candle;
  public CANdleLED(CANdle candle) {
    this.candle = candle;
    
    CANdleConfiguration config = new CANdleConfiguration();

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1;
    this.candle.configAllSettings(config);

    this.candle.setLEDs(0, 0, 100);
  }

  public void setLeds(Integer start, Integer end, Integer r, Integer g, Integer b){
    candle.setLEDs(r, g, b);
  }

  public void end(){
    candle.setLEDs(0, 0, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
