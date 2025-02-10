// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.Lidar;

public class Intake extends SubsystemBase {
  private TalonFX IntakeMotor;
  private Lidar lidar;

  public Intake() {
    IntakeMotor = new TalonFX(Constants.CAN_IDs.IntakeMotorID,"1599-B");
    lidar = new Lidar(new DigitalInput(0));
  }

  public void RunIntake(double Velocity){
    IntakeMotor.set(Velocity);
  }

  public void stop()
  {
    RunIntake(0);
  }

  public double getDistance(){
    return lidar.getDistanceIn();
  }

  @Override
  public void periodic() {
    // This method will be called once sper scheduler run
    SmartDashboard.putNumber("Lidar", getDistance());
  }
}
