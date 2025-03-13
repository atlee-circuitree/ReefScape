// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.BooleanSupplier;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix.ButtonMonitor;
import com.ctre.phoenix.ButtonMonitor.IButtonPressEventHandler;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoOuttakeCommand;
import frc.robot.commands.DriveFoward;
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
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.armExtension;
import frc.robot.subsystems.wrist;



public class RobotContainer {
  
    public final CommandXboxController Player1 = new CommandXboxController(0);
    public final CommandXboxController Player3 = new CommandXboxController(2);
    public final GenericHID Player2 = new GenericHID(1);

    public int SelectArcade = 12;
    public int HumanPlayerArcade = 5;
    public int StartArcade = 4;
    public int RedLeftTopArcade = 11;
    public int RedLeftMiddleArcade =10;
    public int RedLeftBottomArcade = 9;
    public int BlueTopArcade = 6;
    public int BlueMiddleArcade = 7;
    public int BlueBottomArcade = 8;
    public int RedRightTopArcade = 3;
    public int RedRightMiddleArcade = 2;
    public int RedRightBottomArcade = 1;

    private final Intake intake = new Intake();
    private final wrist Wrist = new wrist();
    private final Lights lights = new Lights();
    private final Pivot pivot = new Pivot();
    private final armExtension extension = new armExtension();

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.1).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.1) // Small deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric

    private final Telemetry logger = new Telemetry(Constants.Drive.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

    private AutoFactory autoFactory;
    private final AutoChooser autoChooser;
   
    AutoTrajectory traj;

    public RobotContainer() {
        configureBindings();
        autoFactory = null;
        //autoFactory = drivetrain.createAutoFactory();
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("RedTopScore2Part1", this::RedTopScore2);
        autoChooser.addRoutine("DO NOT USE PIT TEST", this::pitTestAuto);
        autoChooser.addRoutine("DoNothingAuto", this::DoNothingAuto);
        autoChooser.addRoutine("MoveFowardAuto", this::MoveFowardAuto);
        autoChooser.addRoutine("L1FromMiddle", this::L1Auto);
        SmartDashboard.putData("auto", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        autoChooser.addRoutine("MiddleRedAuto", this::MiddleRedAuto);
    
    }

    private void configureBindings() {

        SlewRateLimiter joystickLimiterX = new SlewRateLimiter(2);
        SlewRateLimiter joystickLimiterY = new SlewRateLimiter(2);

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-joystickLimiterX.calculate(joystick.getLeftY()) * Constants.Drive.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickLimiterY.calculate(joystick.getLeftX()) * Constants.Drive.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.Drive.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        JoystickButton Select = new JoystickButton(Player2, SelectArcade);
        JoystickButton HumanPlayer = new JoystickButton(Player2, HumanPlayerArcade);
        JoystickButton Start = new JoystickButton(Player2, StartArcade);
        JoystickButton RedTopLeft = new JoystickButton(Player2, RedLeftTopArcade);
        JoystickButton RedMiddleLeft = new JoystickButton(Player2, RedLeftMiddleArcade);
        JoystickButton RedBottomLeft = new JoystickButton(Player2, RedLeftBottomArcade);
        JoystickButton BlueTop = new JoystickButton(Player2, BlueTopArcade);
        JoystickButton BlueMiddle = new JoystickButton(Player2, BlueMiddleArcade);
        JoystickButton BlueBottom = new JoystickButton(Player2, BlueBottomArcade);
        JoystickButton RedTopRight = new JoystickButton(Player2, RedRightTopArcade);
        JoystickButton RedMiddleRight = new JoystickButton(Player2, RedRightMiddleArcade);
        JoystickButton RedBottomRight = new JoystickButton(Player2, RedRightBottomArcade);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));//James changed from Leftbumper 2/24/2024
        
        /*player1.rightTrigger().whileTrue(new ManualIntake(intake,-.5));
        Player1.leftTrigger().whileTrue(new ManualIntake(intake,0.8));
        Player1.leftBumper().whileTrue(new ManualExtension(extension, -0.2)); //up
        //Player1.rightBumper().whileTrue(new ManualExtension(extension, 0.2)); //down
        Player1.rightBumper().whileTrue(new AprliDrive(drivetrain));
        Player1.y().whileTrue(new ManualWrist(Wrist, -.8));
        Player1.x().whileTrue(new ManualWrist(Wrist,.8)); // goes fowards
        Player1.a().whileTrue(new ManualPivot(pivot,1)); //backward
        Player1.b().whileTrue(new ManualPivot(pivot, -1)); // goes foward
        
        Player1.povUp().toggleOnTrue(new ExtensionCommand(extension, 3.3));
        Player1.povDown().toggleOnTrue(new ExtensionCommand(extension, .5));
        Player1.povLeft().toggleOnTrue(new WristCommand(Wrist, 45));//87.3
        Player1.povRight().toggleOnTrue(new WristCommand(Wrist, 90));*/
        
        //reef lvl 4
        Player3.a().whileTrue(new DriveFoward(drivetrain));
        RedTopLeft.onTrue(new WristCommand(Wrist, 205));
        //bring it back to front
        
        //Manual stuff
        Player3.leftTrigger().whileTrue(new ManualWrist(Wrist, -.8));
        Player3.rightTrigger().whileTrue(new ManualWrist(Wrist, .8));

        Player3.y().whileTrue( 
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(4) 
        .withVelocityY(.2 * (-LimelightHelpers.getTX("limelight-cg")/ Math.abs(LimelightHelpers.getTX("limelight-cg")))) 
        .withRotationalRate(0)));//.until(() -> Math.abs(LimelightHelpers.getTX("limelight-cg")) < 0.3)));

        Player3.x().whileTrue(
        drivetrain.applyRequest(() -> driveRobotCentric
        .withVelocityX(.5) 
        .withVelocityY(0) 
        .withRotationalRate(0)));//.until(() -> -LimelightHelpers.getTA("limelight-cg") == 0)));
 
        //start pos
        Start.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.StartPivot), 
            new WristCommand(Wrist, Constants.Positions.StartWrist) 
        ));
        //human player
        HumanPlayer.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot),
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist)
        ));
        //L3
        RedMiddleLeft.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L3PivotPosition),
            new WristCommand(Wrist, Constants.Positions.L3WristPosition)
        ));
        //L4
        RedTopLeft.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
            new ParallelCommandGroup(
                new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new WristCommand(Wrist, Constants.Positions.L4WristPosition)
                )
            )
        ));

        //L2
        RedBottomLeft.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L2PivotPosition),
            new WristCommand(Wrist, Constants.Positions.L2WristPosition)
        ));


        //manual stuff
        Player1.rightTrigger().whileTrue(new AutoIntakeCommand(intake));//intake
        Player1.leftTrigger().whileTrue(new ManualIntake(intake,0.8));//outtake
        Player1.povDown().whileTrue(new ExtensionCommand(extension, 0.5));
        Player1.povRight().whileTrue(new ManualPivot(pivot, 1)); // backward
        Player1.povLeft().whileTrue(new ManualPivot(pivot, -1)); // foward
        Player1.rightBumper().whileTrue(new ManualExtension(extension, -.5));
        Player1.x().whileTrue(new ManualWrist(Wrist, -.8));
        Player1.y().whileTrue(new ManualWrist(Wrist, .8));


        //coral human player station
        /*Player1.x().toggleOnTrue(new SequentialCommandGroup(
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist),
            new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot)
        
        ));
        //reef lvl 3
        Player1.b().toggleOnTrue(new SequentialCommandGroup(
            new PivotCommand(pivot,Constants.Positions.L3PivotPosition),
            new WristCommand(Wrist, Constants.Positions.L3WristPosition).withTimeout(3),
            new ParallelCommandGroup(
                new WaitCommand(2),
                new WristCommand(Wrist, 20))

            
        ));
        //reef lvl 2
        Player1.a().toggleOnTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L2PivotPosition),
            new WristCommand(Wrist, Constants.Positions.L2WristPosition)
        ));
        //reef lvl 4
        Player1.y().toggleOnTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
            new ParallelCommandGroup(
                new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new WristCommand(Wrist, Constants.Positions.L4WristPosition)
                )
            )
        ));*/
        /*Player1.y().toggleOnTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, 35),
            new WristCommand(Wrist, 210),
            new ExtensionCommand(extension, 2.95),
            new PivotCommand(pivot, 45)
        ));*/

        //low ball
        //1.52 = 6 inches
        Player1.leftBumper().toggleOnTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, 70) // 1.37
        ));
        //climb
        Player1.povUp().toggleOnTrue(new SequentialCommandGroup(
            new WristCommand(Wrist, Constants.Positions.WristClimb),
            new PivotCommand(pivot, Constants.Positions.PivotClimb)
        ));

        //Wrist.setDefaultCommand(new ApplyWristFeedforward(Wrist));

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

    private AutoRoutine RedTopScore2(){
        try {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                autoFactory = drivetrain.createAutoFactoryBlue();
            else    
                autoFactory = drivetrain.createAutoFactory();
        } catch (Exception e) {
            autoFactory = drivetrain.createAutoFactory();
            AutoRoutine routine = autoFactory.newRoutine("blank");
            return routine;
        }

        AutoRoutine routine = autoFactory.newRoutine("RedTopScore2");
        AutoTrajectory RedTopScore2Part1 = routine.trajectory("RedTopScore2Part1");
        AutoTrajectory RedTopScore2Part2 = routine.trajectory("RedTopScore2Part2");
        AutoTrajectory RedTopScore2Part3 = routine.trajectory("RedTopScore2Part3");

        routine.active().onTrue(
            Commands.sequence(
                RedTopScore2Part1.resetOdometry(),
                RedTopScore2Part1.cmd()
            )
        );

        RedTopScore2Part1.atTime("L2").onTrue(new SequentialCommandGroup(
        new WristCommand(Wrist, Constants.Positions.L2WristPosition),
        new PivotCommand(pivot, Constants.Positions.L2PivotPosition)
        ));
        RedTopScore2Part1.atTime("Outake").onTrue(new AutoOuttakeCommand(intake));

        RedTopScore2Part1.done().onTrue(Commands.sequence(
            Commands.waitSeconds(1.5),
            RedTopScore2Part2.cmd()
        ));

        RedTopScore2Part2.atTime("CoralPos").onTrue(new SequentialCommandGroup(
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist),
            new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot)
        ));
        RedTopScore2Part2.atTime("Intake").onTrue(Commands.sequence(
            Commands.waitSeconds(2),
            new AutoIntakeCommand(intake)
            
        ));

        RedTopScore2Part2.done().onTrue(Commands.sequence(

            new AutoIntakeCommand(intake),
            Commands.waitSeconds(.5),
            RedTopScore2Part3.cmd()
            ));


        RedTopScore2Part3.atTime("L1-2").onTrue(Commands.sequence(
            new WristCommand(Wrist, Constants.Positions.L1WristPosition),
            new PivotCommand(pivot, Constants.Positions.L1PivotPosition)
        ));
        RedTopScore2Part3.atTime("Outake-2").onTrue(new AutoOuttakeCommand(intake));

        return routine;
    }

    private AutoRoutine pitTestAuto() {
        autoFactory = drivetrain.createAutoFactory();

        AutoRoutine routine = autoFactory.newRoutine("pit_test_auto");
        AutoTrajectory traj = routine.trajectory("pit_test");
        AutoTrajectory traj2 = routine.trajectory("pit_test2");

        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd(),
            new AutoIntakeCommand(intake),
            traj2.resetOdometry(),
            traj2.cmd()
        ));
        return routine;
    }

    private AutoRoutine DoNothingAuto() {
        AutoRoutine routine = autoFactory.newRoutine("DoNothingAuto");
        

        return routine;

    }

    private AutoRoutine MoveFowardAuto() {
        try {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                autoFactory = drivetrain.createAutoFactoryBlue();
            else    
                autoFactory = drivetrain.createAutoFactory();
        } catch (Exception e) {
            autoFactory = drivetrain.createAutoFactory();
            AutoRoutine routine = autoFactory.newRoutine("blank");
            return routine;
        }
        AutoRoutine routine = autoFactory.newRoutine("MoveFowardAuto");
        AutoTrajectory traj = routine.trajectory("MoveFoward");

        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;

    }

    private AutoRoutine L1Auto() {
        AutoRoutine routine = autoFactory.newRoutine("L1Auto");
        AutoTrajectory traj = routine.trajectory("L1");

        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        traj.atTime("L1").onTrue(Commands.sequence(
            new WristCommand(Wrist, Constants.Positions.L1WristPosition),
            new PivotCommand(pivot, Constants.Positions.L1PivotPosition)
        ));

        traj.atTime("Outtake").onTrue(Commands.sequence(
           new AutoOuttakeCommand(intake)
        ));

        return routine;

    }
    private AutoRoutine MiddleRedAuto(){
        try {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                autoFactory = drivetrain.createAutoFactoryBlue();
            else    
                autoFactory = drivetrain.createAutoFactory();
        } catch (Exception e) {
            autoFactory = drivetrain.createAutoFactory();
            AutoRoutine routine = autoFactory.newRoutine("blank");
            return routine;
        }
        AutoRoutine routine = autoFactory.newRoutine("MiddleAuto");
        AutoTrajectory MiddleRedAuto1 = routine.trajectory("MiddleRedAuto1");
        AutoTrajectory MiddleRedAuto2 = routine.trajectory("MiddleRedAuto2");
        AutoTrajectory MiddleRedAuto3 = routine.trajectory("MiddleRedAuto3");
        AutoTrajectory MiddleRedAuto4 = routine.trajectory("MiddleRedAuto4");


        routine.active().onTrue(
            Commands.sequence(
                MiddleRedAuto1.resetOdometry(),
                MiddleRedAuto1.cmd()
            )
        );
        MiddleRedAuto1.atTime("L4").onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
            new ParallelCommandGroup(
                new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new WristCommand(Wrist, Constants.Positions.L4WristPosition)
                )
            )
        ));
        MiddleRedAuto1.atTime("Outtake").onTrue(new AutoOuttakeCommand(intake));

        MiddleRedAuto1.done().onTrue(Commands.sequence(
            Commands.waitSeconds(1.5),
            MiddleRedAuto2.cmd()
        ));
        return routine;
    }
    

}
