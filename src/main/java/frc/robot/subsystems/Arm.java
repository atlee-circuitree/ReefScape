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

public class Arm extends SubsystemBase {
  TalonFX AngleMotor;
  
  DutyCycleEncoder AngleEncoder;

  double CurrentTicks;
  public double CurrentAngle;

  public Arm() {

    AngleMotor = new TalonFX(Constants.CAN_IDs.AngleID, "1599-B");
    AngleMotor.setNeutralMode(NeutralModeValue.Brake);
    
    AngleEncoder = new DutyCycleEncoder(6);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   CurrentAngle = -CurrentTicks / (.072 / 28) + 60;
  
   SmartDashboard.putNumber("Angle Encoder Degrees", CurrentAngle);
    
   SmartDashboard.putNumber("Angle Encoder Raw", CurrentTicks);

    

  } 
}
