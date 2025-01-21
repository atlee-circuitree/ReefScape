// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armExtension extends SubsystemBase {

  TalonFX extension_left;
  TalonFX extension_right;
  

  

  public armExtension() {

    extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
    extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");
   
    extension_right.setNeutralMode(NeutralModeValue.Brake);
    extension_left.setNeutralMode(NeutralModeValue.Brake);

  }

  public void RunExtension(double Velocity){
    extension_left.set(Velocity);
    extension_right.set(Velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
