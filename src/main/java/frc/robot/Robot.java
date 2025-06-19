// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.armExtension;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
 
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 


    SmartDashboard.putNumber("LimeLight", LimelightHelpers.getTX("limelight-cg"));

    /*try {
      lastResult = LimelightHelpers.getLatestResults("limelight-cg").targetingResults;
    } catch (Exception e) {
      return;
    }*/

    //if (lastResult.valid) {
      //limelight code?? 
    //}

  }

  @Override
  public void disabledInit() {}

  DutyCycleEncoder encoder0 = new DutyCycleEncoder(0);
  DutyCycleEncoder encoder1 = new DutyCycleEncoder(1);
  DutyCycleEncoder encoder2 = new DutyCycleEncoder(2);

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("encoder 0", encoder0.get());
    SmartDashboard.putNumber("encoder 1", encoder1.get());
    SmartDashboard.putNumber("encoder 2", encoder2.get());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
