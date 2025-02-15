package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.wrist;

public class ApplyWristFeedforward extends Command
{        
    wrist m_wrist;

    public ApplyWristFeedforward(wrist wrist) {
        m_wrist = wrist;
        addRequirements(wrist);
    }

  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.feedforward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
