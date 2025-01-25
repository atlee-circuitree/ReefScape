// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput; //for limit switches
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armExtension extends SubsystemBase {

  private final TalonFX extension_left;
  private final TalonFX extension_right;
  
  private final DigitalInput upperLimitSwitch;
  private final DigitalInput lowerLimitSwitch;

  public armExtension(int lowerLimitChannel, int upperLimitChannel) {
    
        extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
        extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");
       
        extension_right.setNeutralMode(NeutralModeValue.Brake);
        extension_left.setNeutralMode(NeutralModeValue.Brake);
    
        upperLimitSwitch = new DigitalInput(upperLimitChannel);
        lowerLimitSwitch = new DigitalInput(lowerLimitChannel);


  }

  public void RunExtension(double Velocity){
    //Check the limit switches before running the motors
    if (Velocity > 0 && upperLimitSwitch.get()) {
      //If extending and the upper limit is reached, stop the motors
      stopExtension();
    } else if (Velocity < 0 && lowerLimitSwitch.get()) {
        //If retracting and the lower limit is reached, stop the motors
        stopExtension();
    } else {
        //Otherwise, run the motors at the specified velocity
        extension_left.set(Velocity);
        extension_right.set(Velocity);
    }
  }

  public void extendToPosition()
  {

  }

  public void stopExtension() {
    extension_left.set(0);
    extension_right.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
