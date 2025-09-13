// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;

public class wrist extends SubsystemBase {
  
  private TalonFX wrist;
  private DutyCycleEncoder wristEncoder;
  private PIDController pid;

  //public double CurrentWristAngle;
  //double CurrentTicks;

  public wrist() {


    wrist = new TalonFX(Constants.CAN_IDs.wrist, "1599-B");

    wrist.setNeutralMode(NeutralModeValue.Brake);



    wristEncoder = new DutyCycleEncoder(Constants.Channels.EncoderChannel);

    pid = new PIDController(Constants.Arm.wristP, Constants.Arm.wristI, Constants.Arm.wristD);
  }

  public double calcFeed()
  {
    //double ang = getAngle();
    // do math
    double out = 0;
    return out;
  }

  public void feedforward()
  {
    RunWrist(calcFeed());
  }

  public void RunWrist(double Velocity){
  
    if (getAngle() <= Constants.Arm.wristThreshold + 2.5 && Velocity < 0) {
      wrist.set(0);
    } else if (getAngle() >= Constants.Arm.upperWristThreshold && Velocity > 0){
      wrist.set(0);
    } else {
      wrist.set(Velocity);
    }
  }

  public void clearPID()
  {
    pid = new PIDController(Constants.Arm.wristP, Constants.Arm.wristI, Constants.Arm.wristD);
    pid.reset();
  }

  public void runToPosition(double deg)
  {
    pid.setSetpoint(deg);
    double out = pid.calculate(getAngle()) - calcFeed();
    RunWrist(out);
    //SmartDashboard.putNumber("WristAngleInput",getAngle());
    SmartDashboard.putNumber("WristAngleOutput",out);
    //SmartDashboard.putNumber("WristAngleSetPoint",deg);*/
  }

  public void stop()
  {
    RunWrist(0);
  }

  public double getAngle() {
    double CurrentTicks = wristEncoder.get();
    if (wristEncoder.get() <= Constants.Arm.wristEncoderOffset) {
      CurrentTicks +=1;
  }
    CurrentTicks = -(CurrentTicks - Constants.Arm.wristEncoderOffset);
    CurrentTicks = 360 - ((CurrentTicks / Constants.Arm.wristRatio) * -360);
    if (CurrentTicks > 330) {
      return 0;      
    }
    return CurrentTicks;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder Get", wristEncoder.get());
    SmartDashboard.putNumber("Wrist Encoder Get w off", wristEncoder.get() - Constants.Arm.wristEncoderOffset);
    SmartDashboard.putNumber("Wrist Degrees", getAngle());
  }
}
