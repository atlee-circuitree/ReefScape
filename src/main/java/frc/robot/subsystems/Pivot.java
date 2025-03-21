// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.CanCoder;
import frc.robot.generated.Pigeon;

public class Pivot extends SubsystemBase {
  private TalonFX pivot_left;
  private TalonFX pivot_right;
  private PIDController pid;
  private Pigeon pivotPigeon;
  private double out;
  private DutyCycleEncoder pivotEncoder;

  public Pivot() {
    pivot_left = new TalonFX(Constants.CAN_IDs.pivotLeft, "1599-B");
    pivot_right = new TalonFX(Constants.CAN_IDs.pivotRight, "1599-B");

    pivot_left.setNeutralMode(NeutralModeValue.Brake);
    pivot_right.setNeutralMode(NeutralModeValue.Brake);

    pivot_left.setInverted(false);
    pivot_right.setInverted(false);
    pid = new PIDController(Constants.Arm.pivotP, Constants.Arm.pivotI, Constants.Arm.pivotD);
    pivotPigeon = new Pigeon(Constants.CAN_IDs.PivotPigeon);
 
    pid = new PIDController(Constants.Arm.pivotP, Constants.Arm.pivotI, Constants.Arm.pivotD);

    pivotEncoder = new DutyCycleEncoder(Constants.Channels.pivotEncoderChannel);
  }

  public void runPivot(double Velocity){
  

    if (getAngleEncoder() <= Constants.Arm.pivotThreshold && Velocity < 0) {
      pivot_left.set(0);
      pivot_right.set(0);
    } else if (getAngleEncoder() >= Constants.Arm.upperPivotThreshold && Velocity > 0){
      pivot_left.set(0);
      pivot_right.set(0);
    } else {
      pivot_left.set(Velocity);
      pivot_right.set(Velocity);
    }

   
  }

  public void clearPID()
  {
  
    pid = new PIDController(Constants.Arm.pivotP, Constants.Arm.pivotI, Constants.Arm.pivotD);
    pid.reset();
  }

  public double getPID(double deg) {

    pid.setSetpoint(deg);
    out = pid.calculate(getAngleEncoder());
    return out;

  }

  public void runToPosition(double deg)
  {
     
    out = getPID(deg);
    SmartDashboard.putNumber("PivotAngleInput",getAngleEncoder());
    SmartDashboard.putNumber("PivotAngleOutput",out);
    SmartDashboard.putNumber("PivotAngleSetPoint",deg);

    runPivot(out);

  }

  public void stop()
  {
    runPivot(0);
  }


  
  public double getAngleEncoder() {
    return (pivotEncoder.get() - Constants.Arm.pivotEncoderOffsetRev) * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Pivot pigeon get", pivotPigeon.getAngle());
    //SmartDashboard.putNumber("Pivot Degrees", getAngle());
    SmartDashboard.putNumber("Pivot Encoder get", pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Encoder Degrees", getAngleEncoder());
    SmartDashboard.putBoolean("Tv",LimelightHelpers.getTV("limelight-cg"));
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight-cg"));
    SmartDashboard.putNumber("TA", LimelightHelpers.getTA("limelight-cg"));
    
    //SmartDashboard.putNumber("Pivot Encoder Raw", CurrentTicks);
  }
}
