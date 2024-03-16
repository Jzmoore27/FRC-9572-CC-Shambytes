// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat.Tuple3;

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
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);
  }

  public void setLeds(Integer start, Integer end, Tuple3<Integer> rgb){
    candle.setLEDs(rgb.get_0(), rgb.get_1(), rgb.get_2(), 0, start, end-start);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
