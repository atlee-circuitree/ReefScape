// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprliDrive extends Command {
  /** Creates a new AprliDrive. */
  private CommandSwerveDrivetrain m_drivetrain;
  private PIDController pid;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public AprliDrive(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drive;
    pid = new PIDController(0.1, 0, 0);
    pid.setSetpoint(0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double out = pid.calculate(LimelightHelpers.getTX("limelight-cg"));
    m_drivetrain.applyRequest(() -> 
        drive.withVelocityX(0.1 * Constants.Drive.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * Constants.Drive.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(out * Constants.Drive.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> 
        drive.withVelocityX(0 * Constants.Drive.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(0 * Constants.Drive.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * Constants.Drive.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //double[] targetCounts = LimelightHelpers.getTargetCount("limelight-cg");
    //return LimelightHelpers.getTargetCount("limelight-cg") == 0;
    //return targetCounts.length == 1;
  }
}
