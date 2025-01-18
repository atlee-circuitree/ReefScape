// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armExtension extends SubsystemBase {

  TalonFX extension_left;
  TalonFX extension_right;
  TalonFX pivot_left;
  TalonFX pivot_right;

  DutyCycleEncoder pivotEncoder;

  public double CurrentPivotAngle;
  double CurrentTicks;

  public armExtension() {

    extension_left = new TalonFX(Constants.CAN_IDs.extension_left, "1599-B");
    extension_right = new TalonFX(Constants.CAN_IDs.extension_right, "1599-B");
    pivot_left = new TalonFX(Constants.CAN_IDs.pivot_left, "1599-B");
    pivot_right = new TalonFX(Constants.CAN_IDs.pivot_right, "1599-B");

    pivot_left.setNeutralMode(NeutralModeValue.Brake);
    pivot_right.setNeutralMode(NeutralModeValue.Brake);
    extension_right.setNeutralMode(NeutralModeValue.Brake);
    extension_left.setNeutralMode(NeutralModeValue.Brake);

    pivotEncoder = new DutyCycleEncoder(Constants.Channels.EncoderChannel);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (pivotEncoder.get() > .8) {

      CurrentTicks = pivotEncoder.get() -1;

    } else {

      CurrentTicks = pivotEncoder.get();

    }

    CurrentPivotAngle =- CurrentTicks / (0.072 / 28) + 60;

    SmartDashboard.putNumber("Pivot Encoder Degrees", CurrentPivotAngle);
    SmartDashboard.putNumber("Pivot Encoder Raw", CurrentTicks);

  }
}
