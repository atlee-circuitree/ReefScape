// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoOuttakeCommand extends Command {
  Intake m_outtake;
  

  public AutoOuttakeCommand(Intake outtake) 
    {
        
        m_outtake = outtake;
        addRequirements(outtake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_outtake.RunIntake(Constants.Arm.outtakeVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_outtake.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_outtake.getCoralDistance() >= 4.5;
  }
}
