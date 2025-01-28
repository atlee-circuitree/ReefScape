// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armExtension;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExtendToPosition extends Command {
  double m_currentExtension;
  double m_targetExtension;
  armExtension m_armExtension;

  public ExtendToPosition(armExtension armExtension, double Extension) {
    
    m_armExtension = armExtension;
    m_targetExtension = Extension;

    addRequirements(armExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_currentExtension = m_armExtension.ReturnCurrentExtension();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_currentExtension = m_armExtension.ReturnCurrentExtension();

    m_armExtension.RunExtensionWithLimits(Constants.ExtensionPID.calculate(m_currentExtension,m_targetExtension));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_armExtension.RunExtensionWithLimits(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Constants.ExtensionPID.calculate(m_currentExtension,m_targetExtension)) <= 0.05) {

      return true;

    }else {

      return false;

    }
    
  }
}
