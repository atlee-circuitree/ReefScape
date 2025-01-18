// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class wrist extends SubsystemBase {
  
  TalonFX wrist;

  DutyCycleEncoder wristEncoder;

  public double CurrentWristAngle;
  double CurrentTicks;

  public wrist() {

    wrist = new TalonFX(Constants.CAN_IDs.wrist, "1599-B");

    wrist.setNeutralMode(NeutralModeValue.Brake);

    wristEncoder = new DutyCycleEncoder(Constants.Channels.EncoderChannel);

  }

  @Override
  public void periodic() {
    
    if (wristEncoder.get() > 0.8) {

      CurrentTicks = wristEncoder.get() -1;

    } else {
      
      CurrentTicks = wristEncoder.get();

    }

    CurrentWristAngle =- CurrentTicks / (0.072 / 28) + 60;

  }
}
