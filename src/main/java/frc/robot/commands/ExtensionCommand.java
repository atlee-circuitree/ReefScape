package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.armExtension;

public class ExtensionCommand extends Command
{
    double m_extension;
    armExtension m_armExtension;

    public ExtensionCommand(armExtension armExtension, double extension) 
    {
        m_armExtension = armExtension;
        m_extension = extension;
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
    m_armExtension.runToPosition(m_extension);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armExtension.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double err = Math.abs(m_armExtension.getExtension() - m_extension);
    return err <= Constants.Arm.extensionThreshold;
  }
}
