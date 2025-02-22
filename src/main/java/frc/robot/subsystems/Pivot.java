// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.Pigeon;

public class Pivot extends SubsystemBase {
  private TalonFX pivot_left;
  private TalonFX pivot_right;
  private DutyCycleEncoder pivotEncoder;
  private PIDController pid;
  private Pigeon pivotPigeon;

  public Pivot() {
    pivot_left = new TalonFX(Constants.CAN_IDs.pivotLeft, "1599-B");
    pivot_right = new TalonFX(Constants.CAN_IDs.pivotRight, "1599-B");

    pivot_left.setNeutralMode(NeutralModeValue.Brake);
    pivot_right.setNeutralMode(NeutralModeValue.Brake);

    pivot_left.setInverted(false);
    pivot_right.setInverted(false);
    pid = new PIDController(Constants.Arm.pivotP, Constants.Arm.pivotI, Constants.Arm.pivotD);
    pivotPigeon = new Pigeon(Constants.CAN_IDs.PivotPigeon);

    pivotEncoder = new DutyCycleEncoder(Constants.Channels.pivotEncoderChannel);

    SmartDashboard.putNumber("PivotP", 0.0);
    pid = new PIDController(Constants.Arm.pivotP, Constants.Arm.pivotI, Constants.Arm.pivotD);
  }

  public void runPivot(double Velocity){
    pivot_left.set(Velocity);
    pivot_right.set(Velocity);
  }

  public void clearPID()
  {
    double p = SmartDashboard.getNumber("PivotP", 0.0);
    pid = new PIDController(p, Constants.Arm.pivotI, Constants.Arm.pivotD);
    pid.reset();
  }

  public void runToPosition(double deg)
  {
    pid.setSetpoint(deg);
    double out = pid.calculate(getAngle());
    runPivot(out);
  }

  public void stop()
  {
    runPivot(0);
  }

  public double getAngle() {
    return pivotPigeon.getAngle() + Constants.Arm.pivotEncoderOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot pigeon get", pivotPigeon.getAngle());
    SmartDashboard.putNumber("Pivot Degrees", getAngle());
    SmartDashboard.putNumber("Pivot Encoder Raw", CurrentTicks);
  }
}
