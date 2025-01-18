// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;
 
import choreo.Choreo;
import choreo.auto.*;
import choreo.trajectory.*;
import choreo.util.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class TestAuto extends SequentialCommandGroup {
   
  CommandSwerveDrivetrain m_drivetrain;
  RobotContainer m_RobotContainer;

  public TestAuto(CommandSwerveDrivetrain Drivetrain, RobotContainer RobotContainer) {
  
    m_drivetrain = Drivetrain;
    m_RobotContainer = RobotContainer;

    addCommands(

      InitialPose("TestAuto", false), // Seed your inital position
      ChoreoPathing("TestAuto", false)
      )
     
    );

  }

  
  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.loadTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    );

  }
 
  private Command InitialPose(String Trajectory, boolean IsRed) {

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.loadTrajectory(Trajectory).getInitialPose()));

  } 

}
