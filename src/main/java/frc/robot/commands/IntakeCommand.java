package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeCommand extends Command {
    Intake m_intake;
    double m_velocity;  
    
    public IntakeCommand(Intake intake, double Velocity) 
    {
        m_intake = intake;
        m_velocity = Velocity;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intake.RunIntake(m_velocity);

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
    // get distance
    // return distance < threshold
  }

}
     

