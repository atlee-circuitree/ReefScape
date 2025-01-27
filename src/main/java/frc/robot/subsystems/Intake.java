// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX IntakeMotor;

  public Intake() {
    IntakeMotor = new TalonFX(Constants.CAN_IDs.IntakeMotorID,"1599-B");

  }

  public void RunIntake(double Velocity){
    IntakeMotor.set(Velocity);
  }

  public void stop()
  {
    RunIntake(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
