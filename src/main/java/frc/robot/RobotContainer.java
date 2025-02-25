// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.TestAuto;
import frc.robot.commands.ApplyWristFeedforward;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoOuttakeCommand;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.ManualExtension;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.WristCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.armExtension;
import frc.robot.subsystems.wrist;
import frc.robot.commands.AprliDrive;


public class RobotContainer {
  
    public final CommandXboxController Player1 = new CommandXboxController(0);
    public final CommandXboxController Player2 = new CommandXboxController(1);

    private final Intake intake = new Intake();
    private final wrist Wrist = new wrist();
    private final Lights lights = new Lights();
    private final Pivot pivot = new Pivot();
    private final armExtension extension = new armExtension();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.Drive.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
   

    AutoTrajectory traj;

    public RobotContainer() {
        configureBindings();
            autoFactory = drivetrain.createAutoFactory();
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Example Routine", this::FirstAuto);
        SmartDashboard.putData(autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));//James changed from Leftbumper 2/24/2024
        
        Player1.rightTrigger().whileTrue(new ManualIntake(intake,-.5));
        Player1.leftTrigger().whileTrue(new ManualIntake(intake,0.8));
        Player1.leftBumper().whileTrue(new ManualExtension(extension, -0.2)); //up
        //Player1.rightBumper().whileTrue(new ManualExtension(extension, 0.2)); //down
        Player1.rightBumper().whileTrue(new AprliDrive(drivetrain));
        Player1.y().whileTrue(new ManualWrist(Wrist, -.5));
        Player1.x().whileTrue(new ManualWrist(Wrist,.5)); // goes fowards
        Player1.a().whileTrue(new ManualPivot(pivot,1)); //backward
        Player1.b().whileTrue(new ManualPivot(pivot, -1)); // goes foward
        Player1.povUp().toggleOnTrue(new ExtensionCommand(extension, 3.3));
        Player1.povDown().toggleOnTrue(new ExtensionCommand(extension, .5));
        Player1.povLeft().toggleOnTrue(new WristCommand(Wrist, 45));//87.3
        Player1.povRight().toggleOnTrue(new WristCommand(Wrist, 90));
        
        

        //coral human player station
        Player2.a().toggleOnTrue(new SequentialCommandGroup(
            new WristCommand(Wrist, 12),
            new PivotCommand(pivot, 36)
        
        ));
        //reef lvl 3
        Player2.b().toggleOnTrue(new SequentialCommandGroup(
        new WristCommand(Wrist, 166),
        new PivotCommand(pivot, 40)
        ));
        //reef lvl 2
        Player2.x().toggleOnTrue(new SequentialCommandGroup(
        new WristCommand(Wrist, 4),
        new PivotCommand(pivot, 26)
        ));

        //Wrist.setDefaultCommand(new ApplyWristFeedforward(Wrist));
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-joystick.getLeftY() * Constants.Drive.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.Drive.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.Drive.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
          //  point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))

        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return null;
        //autoFactory = this.drivetrain.createAutoFactory();

        //return Commands.sequence(
        //autoFactory.resetOdometry("TestRun"), // 
        //autoFactory.trajectoryCmd("TestRun") 
        //);
        /*AutoRoutine routine = autoFactory.newRoutine("taxi");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("far on score");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );
        //driveToMiddle.atTime("Marker").onTrue(new WristCommand(Wrist, 45));
        return routine.cmd();

        //return Commands.print("No autonomous command configured");*/ 
    }

    private AutoRoutine FirstAuto() {
        AutoRoutine routine = autoFactory.newRoutine("taxi");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("far on score");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );
        //driveToMiddle.atTime("Marker").onTrue(new WristCommand(Wrist, 45));
        return routine;
    }
    
}
