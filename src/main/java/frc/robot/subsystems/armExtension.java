// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.CanCoder;
import edu.wpi.first.math.controller.PIDController;

public class armExtension extends SubsystemBase {

  private final TalonFX extension_left;
  private final TalonFX extension_right;
  
  private final CanCoder extensionCanCoder;

  double CurrentTicks;
  public double CurrentExtension;
  private PIDController pid;



  public armExtension() {
    
        extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
        extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");
       
        extension_right.setNeutralMode(NeutralModeValue.Brake);
        extension_left.setNeutralMode(NeutralModeValue.Brake);
        extensionCanCoder = new CanCoder(Constants.CAN_IDs.ExtensionCANCoder);   

        SmartDashboard.putNumber("ArmP", 0.1);
        SmartDashboard.putNumber("ArmI", 0.0);
        SmartDashboard.putNumber("ArmD", 0.0);
        pid = new PIDController(SmartDashboard.getNumber("ArmP", 0.1), SmartDashboard.getNumber("ArmI", 0), SmartDashboard.getNumber("ArmD", 0));
      }

  public void stop() {
    extension_left.set(0);
    extension_right.set(0);
  }

  public double getExtension(){
    return extensionCanCoder.getDistance() + Constants.Arm.armEncoderOffset;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extension get", extensionCanCoder.getDistance());
    SmartDashboard.putNumber("Extension degree", getExtension());
    
  }

  public double ReturnCurrentExtension() {
    return CurrentExtension;
  }
  
  public void RunExtension(double Speed){
    extension_left.set(-Speed);
    extension_right.set(-Speed);
  }

  public void clearPID()
  {
    pid = new PIDController(SmartDashboard.getNumber("ArmP", 0.1), SmartDashboard.getNumber("ArmI", 0), SmartDashboard.getNumber("ArmD", 0));
    pid.reset();
  }

  public void runToPosition(double Distance)
  {
    pid.setSetpoint(Distance);
    double out = pid.calculate(getExtension());
    RunExtension(out);
  }
}
