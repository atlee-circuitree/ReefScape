// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class AutoIntakeCommand extends Command {
  Intake m_intake;

  public AutoIntakeCommand(Intake intake) {
    SmartDashboard.putNumber("Coral threshold autointake", 2);
    SmartDashboard.putNumber("Algee threshold autointake", 5);
    m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.RunIntake(Constants.Arm.intakeVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getCoralDistance() <= SmartDashboard.getNumber("Coral threshold autointake", 2);
  }
}
