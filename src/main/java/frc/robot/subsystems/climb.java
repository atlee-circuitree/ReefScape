// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climb extends SubsystemBase {
  
  TalonFX climb;

  DutyCycleEncoder climbEncoder;

  public double CurrentclimbAngle;
  double CurrentTicks;

  public climb() {

    climb = new TalonFX(Constants.CAN_IDs.climb, "1599-B");

    climb.setNeutralMode(NeutralModeValue.Brake);

    climbEncoder = new DutyCycleEncoder(Constants.Channels.EncoderChannel);

  }

  @Override
  public void periodic() {
    
    if (climbEncoder.get() > 0.8) {

      CurrentTicks = climbEncoder.get() -1;

    } else {
      
      CurrentTicks = climbEncoder.get();

    }

    CurrentclimbAngle =- CurrentTicks / (0.072 / 28) + 60;

  }
}

