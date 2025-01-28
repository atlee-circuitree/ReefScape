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

public class Pivot extends SubsystemBase {
  private TalonFX pivot_left;
  private TalonFX pivot_right;
  private DutyCycleEncoder pivotEncoder;

  public Pivot() {
    pivot_left = new TalonFX(Constants.CAN_IDs.pivotLeft, "1599-B");
    pivot_right = new TalonFX(Constants.CAN_IDs.pivotRight, "1599-B");

    pivot_left.setNeutralMode(NeutralModeValue.Brake);
    pivot_right.setNeutralMode(NeutralModeValue.Brake);
    pivot_left.setInverted(false);
    pivot_right.setInverted(false);

    pivotEncoder = new DutyCycleEncoder(Constants.Channels.pivotEncoderChannel);
  }

  public void runPivot(double Velocity){
    pivot_left.set(Velocity);
    pivot_right.set(Velocity);
  }

  public void runToPosition()
  {
    
  }

  public void stop()
  {
    runPivot(0);
  }

  public double getAngle() {
    double CurrentTicks = pivotEncoder.get();
    return CurrentTicks / (0.072 / 28) + 60;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ang = getAngle();
    SmartDashboard.putNumber("Pivot Encoder Degrees", ang);
    //SmartDashboard.putNumber("Pivot Encoder Raw", CurrentTicks);
  }
}
