package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class PivotCommand extends Command
{
    double m_position;
    Pivot m_pivot;
    int heartbeat;

    public PivotCommand(Pivot pivot, double position) 
    {
        m_pivot = pivot;
        m_position = position;
        
        addRequirements(pivot);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_pivot.clearPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    m_pivot.runToPosition(m_position);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_pivot.getPID(m_position) < .1 && m_pivot.getPID(m_position) > -.1) {
      return true;
    }else{
      return false;
    } 
  }
}
