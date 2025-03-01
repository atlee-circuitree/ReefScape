// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.CanRange;

public class Intake extends SubsystemBase {
  private TalonFX IntakeMotor;
  private CanRange m_CoralCanRange;
  //private CanRange m_AlgeeCanRange;

  public Intake() {
    IntakeMotor = new TalonFX(Constants.CAN_IDs.IntakeMotorID,"1599-B");
    m_CoralCanRange = new CanRange(Constants.CAN_IDs.CoralCANRange);
  }

  public void RunIntake(double Velocity){
    IntakeMotor.set(Velocity);
  }

  public void stop()
  {
    RunIntake(0);
  }

  public double getCoralDistance(){
    return m_CoralCanRange.getDistance(Inches);
  }

  /*public double getAlgeeDistance(){
    return m_AlgeeCanRange.getDistance(Inches);
  }*/

  @Override
  public void periodic() {
    // This method will be called once sper scheduler run
    SmartDashboard.putNumber("Coral CanRange", getCoralDistance());
    //SmartDashboard.putNumber("Algee CanRange", getAlgeeDistance());
    SmartDashboard.putBoolean("All", DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
  }
}
