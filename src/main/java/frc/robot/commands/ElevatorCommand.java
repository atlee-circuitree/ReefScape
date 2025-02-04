package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.armExtension;

public class ElevatorCommand extends Command
{
    armExtension m_arm;
    double m_position;

    public ElevatorCommand(armExtension arm, double position) 
    {
        m_arm = arm;
        m_position = position;
        addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_arm.stop();
    }

    @Override
    public void execute() 
    {
        m_arm.runToPosition(m_position);
    }

    @Override
    public void initialize() 
    {
        m_arm.clearPID();
    }

    @Override
    public boolean isFinished() 
    {
        double err = Math.abs(m_arm.getDistance() - m_position);
        return err <= Constants.Arm.armThreshold;
    }
    
}
