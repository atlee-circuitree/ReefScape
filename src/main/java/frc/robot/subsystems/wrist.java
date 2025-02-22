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
    double ang = getAngle();
    // do math
    double out = 0;
    return out;
  }

  public void feedforward()
  {
    RunWrist(calcFeed());
  }

  public void RunWrist(double Velocity){
    wrist.set(Velocity);
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
    RunWrist(-out);
  }

  public void stop()
  {
    RunWrist(0);
  }

  public double getAngle() {
    double CurrentTicks = wristEncoder.get() - Constants.Arm.wristEncoderOffset;
    return (CurrentTicks / Constants.Arm.wristRatio) * -360;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder Get", wristEncoder.get());
    SmartDashboard.putNumber("Wrist Degrees", getAngle());
  }
}
