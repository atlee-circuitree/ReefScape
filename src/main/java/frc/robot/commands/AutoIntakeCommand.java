// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.generated.Lidar;



public class AutoIntakeCommand extends Command {
  Intake m_intake;

  public AutoIntakeCommand(Intake intake) {
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
    if (m_intake.getDistance() <= 2){
      m_intake.stop();
    }else{
      m_intake.RunIntake(Constants.Arm.intakeVelocity);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;


  }
}
