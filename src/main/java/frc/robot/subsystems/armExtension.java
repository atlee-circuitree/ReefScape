// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput; //for limit switches
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armExtension extends SubsystemBase {

  private final TalonFX extension_left;
  private final TalonFX extension_right;
  
  private final DutyCycleEncoder extensionEncoder;
  private final DutyCycleEncoder upperMaxExtension;
  private final DutyCycleEncoder lowerMaxExtension;

  double CurrentTicks;
  public double CurrentExtension;


  public armExtension() {
    
        extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
        extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");
       
        extension_right.setNeutralMode(NeutralModeValue.Brake);
        extension_left.setNeutralMode(NeutralModeValue.Brake);

        extensionEncoder = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);

        upperMaxExtension = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);
        lowerMaxExtension = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);
        
        


  }

  public void RunExtension(double Velocity){

    //Check the limit switches before running the motors
    if (Velocity > 0 && upperMaxExtension.get() == 100) {
      //If extending and the upper limit is reached, stop the motors
      stopExtension();
    } else if (Velocity < 0 && lowerMaxExtension.get() == 0) {
        //If retracting and the lower limit is reached, stop the motors
        stopExtension();
    } else {
        //Otherwise, run the motors at the specified velocity
        extension_left.set(Velocity);
        extension_right.set(Velocity);
    }
  }

  public void stopExtension() {
    extension_left.set(0);
    extension_right.set(0);
  }

  public double getExtension(){
    CurrentTicks = extensionEncoder.get();
    return CurrentTicks / (0.072 / 28) + 60;
  }

  @Override
  public void periodic() {

    CurrentExtension = -CurrentTicks / (0.072 / 28) + 60;

    double ang = getExtension();
    SmartDashboard.putNumber("Extension Encoder degree", ang);

    
  }

  public double ReturnCurrentExtension() {
    return CurrentExtension;
  }
  
  public void RunExtensionWithLimits(double Speed){
    extension_left.set(-Speed);
    extension_right.set(-Speed);
  }
}
