package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.wrist;

public class WristCommand extends Command {
    
    double m_position;
    wrist m_wrist;
    int heartbeat;

    public WristCommand(wrist wrist, double position) {
        m_position = position;
        m_wrist = wrist;
        heartbeat = 0;
        addRequirements(wrist);
    }

  @Override
  public void initialize() 
  {
    m_wrist.clearPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.runToPosition(m_position);
    heartbeat++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double err = Math.abs(m_wrist.getAngle() - m_position);
    return err <= Constants.Arm.wristThreshold;
  }
}


