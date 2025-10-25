// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.cert.Extension;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.CanCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;

public class armExtension extends SubsystemBase {
  
  private TalonFX extension_left;
  private TalonFX extension_right;
  private PIDController pid;
  private CanCoder ExtensionCanCoder;
  private double CurrentTicks;

  //public double CurrentWristAngle;
  //double CurrentTicks;

  public armExtension() {

    extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
    extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");

    extension_left.setNeutralMode(NeutralModeValue.Brake);
    extension_right.setNeutralMode(NeutralModeValue.Brake);

    ExtensionCanCoder = new CanCoder(Constants.CAN_IDs.ExtensionCANCoder);
    pid = new PIDController(Constants.Arm.ArmP, Constants.Arm.ArmI, Constants.Arm.ArmD);
    pid.setIZone(Constants.Arm.ArmIZone);
  }

  public void RunExtension(double Velocity){
    extension_left.set(-Velocity);
    extension_right.set(-Velocity);
  }

  public void clearPID()
  {
    pid = new PIDController(Constants.Arm.ArmP, Constants.Arm.ArmI, Constants.Arm.ArmD);
    pid.reset();
  }

  public void runToPosition(double deg)
  {
    pid.setSetpoint(deg);
    double out = pid.calculate(getExtension ());
    RunExtension(-out);
  }

  public void stop()
  {
    RunExtension(0);
  }

  public double getExtension() {

    CurrentTicks = ExtensionCanCoder.getDistance() - Constants.Arm.armEncoderOffset;
    return CurrentTicks;
  }

  public static double getExtension(CanCoder ExtensionCanCoder) {
    double CurrentTicks;
    CurrentTicks = ExtensionCanCoder.getDistance() - Constants.Arm.armEncoderOffset;
    if (CurrentTicks < 0) {
      return 0;
    }
    return CurrentTicks + .15;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("extension Encoder Get", ExtensionCanCoder.getDistance());
    SmartDashboard.putNumber("Extension Encoder With Offset", getExtension());
  }
}
