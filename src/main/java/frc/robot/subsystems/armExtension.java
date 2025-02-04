// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput; //for limit switches
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

public class armExtension extends SubsystemBase {

  private final TalonFX extension_left;
  private final TalonFX extension_right;
  
  private final DutyCycleEncoder extensionEncoder;
  private final DutyCycleEncoder upperMaxExtension;
  private final DutyCycleEncoder lowerMaxExtension;

  double CurrentTicks;
  public double CurrentExtension;
  private PIDController pid;



  public armExtension() {
    
        extension_left = new TalonFX(Constants.CAN_IDs.extensionLeft, "1599-B");
        extension_right = new TalonFX(Constants.CAN_IDs.extensionRight, "1599-B");
       
        extension_right.setNeutralMode(NeutralModeValue.Brake);
        extension_left.setNeutralMode(NeutralModeValue.Brake);

        extensionEncoder = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);

        upperMaxExtension = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);
        lowerMaxExtension = new DutyCycleEncoder(Constants.Channels.armExtensionEncoderChannel);
        


        SmartDashboard.putNumber("ArmP", 0.0);
        SmartDashboard.putNumber("ArmI", 0.0);
        SmartDashboard.putNumber("ArmD", 0.0);
        pid = new PIDController(Constants.Arm.armP, Constants.Arm.armI, Constants.Arm.armD);
  }

  public void stop() {
    extension_left.set(0);
    extension_right.set(0);
  }

  public double getExtension(){
    CurrentTicks = extensionEncoder.get();
    return CurrentTicks / (0.072 / 28) + 60;
  }

  @Override
  public void periodic() {

    

    SmartDashboard.putNumber("Extension Encoder get", extensionEncoder.get());
    SmartDashboard.putNumber("Extension Encoder degree", getDistance());
    


    
  }

  public double ReturnCurrentExtension() {
    return CurrentExtension;
  }
  
  public void RunExtension(double Speed){
    extension_left.set(-Speed);
    extension_right.set(-Speed);
  }


  public double getDistanceToAprilTag(){

    HashMap<Double, Double> AprilTags = new HashMap<>();
    AprilTags.put(12d, 58.5d);
    AprilTags.put(13d, 58.5d);
    AprilTags.put(1d, 58.5d);
    AprilTags.put(2d, 58.5d);
    AprilTags.put(6d, 12d);
    AprilTags.put(7d, 12d);
    AprilTags.put(8d, 12d);
    AprilTags.put(9d, 12d);
    AprilTags.put(10d, 12d);
    AprilTags.put(11d, 12d);
    AprilTags.put(17d, 12d);
    AprilTags.put(18d, 12d);
    AprilTags.put(19d, 12d);
    AprilTags.put(20d, 12d);
    AprilTags.put(21d, 12d);
    AprilTags.put(22d, 12d);

    double ty = LimelightHelpers.getTY("cg");

    double targetOffsetAngle_Vertical = ty;

    double AprilTagID = /* armExtension.getAprilTagId() */ 0;

    double tagHeightInches = AprilTags.get(AprilTagID);

    double limelightMountAngleDegrees = 0;

    double limelightHeightInches = 12;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);


    double distanceFromLimelightToGoalInches = -1;

    /* making a list of all april tag unique ids and depending on which april tag it sees
     * and if list contains the april tag that the limelight sees we will set the blue or red tag heights 
     * equal to tagHeightInches
     */
 
    distanceFromLimelightToGoalInches = (tagHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
 
    double distanceFromLimelighttoGoalMeters = distanceFromLimelightToGoalInches / 39.37;
 
    return distanceFromLimelighttoGoalMeters;
  }

  public void clearPID()
  {
    double p = SmartDashboard.getNumber("armP", 0.0);
    double i = SmartDashboard.getNumber("armI", 0.0);
    double d = SmartDashboard.getNumber("armD", 0.0);
    pid = new PIDController(p, i, d);
    pid.reset();
  }

  public double getDistance()
  {
    double CurrentTicks = extensionEncoder.get() - Constants.Arm.armEncoderOffset;
    return (CurrentTicks / Constants.Arm.extensionRatio) * 360;
  }

  public void runToPosition(double Distance)
  {
    pid.setSetpoint(Distance);
    double out = pid.calculate(getDistance());
    RunExtension(out);
  }
}
