// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
       
        
        extensionCanCoder = new CanCoder(Constants.CAN_IDs.ExtensionCANCoder);   

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = extensionCanCoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Regular PIDs
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kV = 0.25; 
        slot0Configs.kP = 0.12;
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0; 
        slot0Configs.kS = 0;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 4.5; // 3 // 4.5
        motionMagicConfigs.MotionMagicAcceleration = 4; // 3 // 4
        motionMagicConfigs.MotionMagicJerk = 0; // 0


        extension_left.getConfigurator().apply(talonFXConfigs, 0.050);
        extension_right.getConfigurator().apply(talonFXConfigs, 0.050);
        extension_right.setNeutralMode(NeutralModeValue.Brake);
        extension_left.setNeutralMode(NeutralModeValue.Brake);
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
  
  public void RunExtension(double Speed){
    extension_left.set(-Speed);
    extension_right.set(-Speed);
  }



  public void runToPosition(double Distance)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(Distance);
    extension_left.setControl(request.withPosition(extensionCanCoder.getDistance()));
    extension_right.setControl(request.withPosition(extensionCanCoder.getDistance()));
  }
}
