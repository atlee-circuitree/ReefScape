// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpeedToggleCommand extends Command {
  /** Creates a new SpeedToggleCommand. */
  public SpeedToggleCommand() { 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Speed Boolean", Constants.Drive.SpeedToggle);
    SmartDashboard.putNumber("Speed", Constants.Drive.Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.Drive.SpeedToggle) {
      Constants.Drive.SpeedToggle = false;
      Constants.Drive.Speed = Constants.Drive.minSpeed;
      SmartDashboard.putBoolean("Speed Boolean", Constants.Drive.SpeedToggle);
      SmartDashboard.putNumber("Speed", Constants.Drive.Speed);
    } else {
      Constants.Drive.Speed = Constants.Drive.maxSpeed;
      Constants.Drive.SpeedToggle = true;
      SmartDashboard.putBoolean("Speed Boolean", Constants.Drive.SpeedToggle);
      SmartDashboard.putNumber("Speed", Constants.Drive.Speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
