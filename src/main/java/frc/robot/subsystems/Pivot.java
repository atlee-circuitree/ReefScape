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
  TalonFX pivot_left;
  TalonFX pivot_right;
  /** Creates a new Pivot. */

  DutyCycleEncoder pivotEncoder;

  public double CurrentPivotAngle;
  double CurrentTicks;

  public Pivot() {

    pivot_left = new TalonFX(Constants.CAN_IDs.pivot_left, "1599-B");
    pivot_right = new TalonFX(Constants.CAN_IDs.pivot_right, "1599-B");

    pivot_left.setNeutralMode(NeutralModeValue.Brake);
    pivot_right.setNeutralMode(NeutralModeValue.Brake);
    pivot_left.setInverted(false);
    pivot_right.setInverted(false);
  }

  public void RunPivot(double Velocity){
    pivot_left.set(Velocity);
    pivot_right.set(Velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
      CurrentTicks = pivotEncoder.get();

    

    CurrentPivotAngle =- CurrentTicks / (0.072 / 28) + 60;

    SmartDashboard.putNumber("Pivot Encoder Degrees", CurrentPivotAngle);
    SmartDashboard.putNumber("Pivot Encoder Raw", CurrentTicks);

  }
}
