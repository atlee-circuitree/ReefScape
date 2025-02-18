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
        pid = new PIDController(SmartDashboard.getNumber("armP", 0), SmartDashboard.getNumber("armI", 0), SmartDashboard.getNumber("armD", 0));
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
    pid = new PIDController(SmartDashboard.getNumber("armP", 0), SmartDashboard.getNumber("armI", 0), SmartDashboard.getNumber("armD", 0));
    pid.reset();
  }

  public void runToPosition(double Distance)
  {
    pid.setSetpoint(Distance);
    SmartDashboard.putNumber("data", pid.getSetpoint());
    double out = pid.calculate(getExtension());
    SmartDashboard.putNumber("datat2", out);
    SmartDashboard.putNumber("data3", getExtension());
    RunExtension(out);
  }
}
