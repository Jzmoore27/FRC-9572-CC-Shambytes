// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveDriveSub;

public class SwerveJoystickCmd extends Command {

  private final SwerveDriveSub swerveDrive;
  private final LimeLight limelight;
  private Double xSpeed, ySpeed, tSpeed;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean>  zeroButton, switchSpds, lime;
  private Boolean fieldRelative = true, zpressed = false, spressed = false;
  private SlewRateLimiter xLimiter, yLimiter, tLimiter;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveDriveSub swerveDrive, LimeLight limelight,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> switchFunction, Supplier<Boolean> zeroFunction, Supplier<Boolean> limeFunction) {
    this.swerveDrive = swerveDrive;
    this.limelight=limelight;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.lime = limeFunction;
    this.fieldRelative = true;
    this.turningSpdFunction = turningSpdFunction;
    this.zeroButton = zeroFunction;
    this.switchSpds = switchFunction;
    this.xLimiter = new SlewRateLimiter(1);
    this.yLimiter = new SlewRateLimiter(1);
    this.tLimiter = new SlewRateLimiter(3);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fieldRelative = true;
    swerveDrive.setMotorBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      xSpeed = xLimiter.calculate(xSpdFunction.get());
      ySpeed = yLimiter.calculate(ySpdFunction.get());
      tSpeed = tLimiter.calculate(turningSpdFunction.get());
      if(lime.get()&&limelight.getY()>0.0){
        swerveDrive.drive(new ChassisSpeeds(-0.2-xSpeed, -ySpeed, (-limelight.getX()/60)+tSpeed));
      }
      else if(fieldRelative){
        swerveDrive.driveFieldOriented(new ChassisSpeeds(-xSpeed, -ySpeed, tSpeed));
      }
      else{
        swerveDrive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, tSpeed));
      }
      SmartDashboard.putBoolean("btn", lime.get());
      if(switchSpds.get()){
        if(!spressed){
          spressed = true;
          fieldRelative = fieldRelative ? false : true;
        }
      }
      else{
        spressed = false;
      }

      if(zeroButton.get()){
        if(!zpressed){
          zpressed = true;
          swerveDrive.zeroGyro();
        }
      }
      else{
        zpressed = false;
      }
    SmartDashboard.putBoolean("fieldOriented", fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
