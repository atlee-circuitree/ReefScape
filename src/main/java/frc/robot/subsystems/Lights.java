// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  

  Spark blinkin;

  public Lights() {

    blinkin = new Spark(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  public void SetColor(double color) {

    blinkin.set(color);

  }
}
