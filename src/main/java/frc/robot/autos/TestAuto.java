// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;



import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAuto extends Command {
  AutoFactory autoFactory;
  CommandSwerveDrivetrain drivetrain;
  AutoTrajectory traj;
  /** Creates a new Autos. */
  public TestAuto(CommandSwerveDrivetrain drivetrain) {
     // The drive subsystem
     this.drivetrain = drivetrain;
    autoFactory = this.drivetrain.createAutoFactory();

    // Use addRequirements() here to declare subsystem dependencies.
  }

 public Command testpath() {
  return Commands.sequence(
    new InstantCommand(
      () -> drivetrain.resetPose(new Pose2d(0,0, new Rotation2d(0.)))
    ),
    autoFactory.trajectoryCmd("TestRun"));
 }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}