package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.armExtension;

public class ExtensionCommand extends Command
{
    double feet;
    double m_position;
    armExtension m_armExtension;

    public ExtensionCommand(armExtension armExtension, double position) 
    {
        
        //feet = position * (6/1.52)
        m_position = position;
        m_armExtension = armExtension;
        addRequirements(armExtension);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_armExtension.clearPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armExtension.runToPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armExtension.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double err = Math.abs(m_armExtension.getExtension() - m_position);
    return err >= Constants.Arm.extensionThreshold;
  }
}
