// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.ManualWrist;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.SpeedToggleCommand;
import frc.robot.commands.WristCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Pivot;
//import frc.robot.subsystems.ReefCentering;
import frc.robot.subsystems.armExtension;
import frc.robot.subsystems.wrist;



public class RobotContainer {
    //private ReefCentering reefCentering = new ReefCentering(drive);
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
    //private final Lights lights = new Lights(); not used
    private final Pivot pivot = new Pivot();
    private final armExtension extension = new armExtension();

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.025).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.04) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.Drive.MaxSpeed * 0.025).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.04) // Small deadband
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
        autoChooser.addRoutine("DO NOT USE PIT TEST", this::pitTestAuto);
        autoChooser.addRoutine("DoNothingAuto", this::DoNothingAuto);
        autoChooser.addRoutine("MoveFowardAuto", this::MoveFowardAuto);
        autoChooser.addRoutine("L1FromMiddle", this::L1Auto);
        SmartDashboard.putData("auto", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        autoChooser.addRoutine("LimelightMiddle",this::LimelightMiddleAuto);
        autoChooser.addRoutine("BottomLimelight",this::BottomLimelightAuto);
        autoChooser.addRoutine("LowBallAuto", this::LowBallAuto);
        autoChooser.addRoutine("NewLimelightAUTOMID", this::NewLimelightAUTO);
    
    }
//Didnt finish 
    /*public Command centerWithAprilTag() {
    return new InstantCommand(() -> {
        double tx = LimelightHelpers.getTX("limelight-cg") + Constants.Drive.TagOffset;
        double  strafePower = -tx * Constants.Drive.AprilStrafeCoeff;
        strafePower = Math.max(-0.5, Math.min(0.5, strafePower));
        drivetrain.applyRequest(() -> driveRobotCentric
            .withVelocityX(0.8)
            .withVelocityY(strafePower)
            .withRotationalRate(0.0)
            ).until -> true;
    }
}*/

    /*public double getTargetTx(boolean isRight)
    {
        double ta = LimelightHelpers.getTA("limelight-cg");
        double targetTx = -1.61 - 3.76 * ta + 0.188 * Math.pow(ta,2);
        if (!isRight)
            targetTx *= -1;
        double tx = LimelightHelpers.getTX("limelight-cg") + Constants.Drive.TagOffset;
        return (targetTx - tx) * Constants.Drive.AprilStrafeCoeff;
    }*/

    public double getTargetTx(boolean isRight)
    {
        if (!LimelightHelpers.getTV("limelight-cg")) {
            return 0.0;
        }
        double ta = LimelightHelpers.getTA("limelight-cg");
        double targetTx = -1.61 -3.76 *ta +0.188 * Math.pow(ta,2);
        if (!isRight) targetTx *= -1;
        
        double tx = LimelightHelpers.getTX("limelight-cg");

        return (targetTx - tx) * Constants.Drive.AprilStrafeCoeff + Constants.Drive.TagOffset;
    }

    public double centerWithAprilTag()
    {
        double tx = LimelightHelpers.getTX("limelight-cg");
            
        double alignmentOffset = -tx; 
            
        return alignmentOffset * Constants.Drive.AprilStrafeCoeff;
        

    }


    private void configureBindings() {

        //SlewRateLimiter joystickLimiterX = new SlewRateLimiter(2); both not used
        //SlewRateLimiter joystickLimiterY = new SlewRateLimiter(2);

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(joystick.getLeftY()) * ((Constants.Drive.MaxSpeed) / Constants.Drive.Speed)) // Drive forward with negative Y (forward)
                    .withVelocityY(-(joystick.getLeftX()) * ((Constants.Drive.MaxSpeed) / Constants.Drive.Speed)) // Drive left with negative X (left)
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
        //JoystickButton RedBottom = new JoystickButton(Player2, BlueBottomArcade); not used
        JoystickButton RedTopRight = new JoystickButton(Player2, RedRightTopArcade);
        //JoystickButton RedMiddleRight = new JoystickButton(Player2, RedRightMiddleArcade); not used
        JoystickButton RedBottomRight = new JoystickButton(Player2, RedRightBottomArcade);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
         
        
        //reef lvl 4
        
        Player3.a().whileTrue(new DriveFoward(drivetrain));


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
        /*RedBottomRight.onTrue(new SequentialCommandGroup(
             drivetrain.applyRequest(() -> driveRobotCentric
                 .withVelocityX(0.8) 
                 .withVelocityY(getTargetTx(true)) 
                 .withRotationalRate(0)
             ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),

             drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(1)
                .withVelocityY(0)
                .withRotationalRate(0)
             ).withTimeout(.5),
             drivetrain.applyRequest(() -> driveRobotCentric
                 .withVelocityX(0) 
                 .withVelocityY(0) 
                 .withRotationalRate(0)

             ).until(() -> true),  
            new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
            new ParallelCommandGroup(
                new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new WristCommand(Wrist, Constants.Positions.L4WristPosition)
                )
            )
        ));*/

        //L2
        RedBottomLeft.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.L2PivotPosition)//,
            //new WristCommand(Wrist, Constants.Positions.L2WristPosition)
        ));
        //bring elevator back down from L4
        Select.onTrue(new ParallelCommandGroup(
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist),
            new ExtensionCommand(extension, Constants.Positions.bringExtensionDown)
        ));
        //low ball
        BlueMiddle.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.LowBallPivot)
            //new WristCommand(Wrist, Constants.Positions.LowBallWrist)
        ));
        //high ball
        BlueTop.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.HighBallPivot)
            //new WristCommand(Wrist, Constants.Positions.HighBallWrist)
        ));
        //climb
        RedBottomRight.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.PivotClimb),
            new WristCommand(Wrist, Constants.Positions.WristClimb)
        ));
        //barge
        /*RedTopRight.onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.BargePivot),
            new ExtensionCommand(extension, Constants.Positions.BargeExtenstion)
        ));*/


        //manual stuff
        Player1.rightTrigger().whileTrue(new ManualIntake(intake,-1));//intake
        Player1.leftTrigger().whileTrue(new ManualIntake(intake,1));//outtake
        Player1.x().onTrue(new SpeedToggleCommand());
        //Player1.povDown().whileTrue(new ExtensionCommand(extension, 0.5));
        Player1.povRight().whileTrue(new ManualPivot(pivot, 1)); // backward
        Player1.povLeft().whileTrue(new ManualPivot(pivot, -1)); // foward
        //Player1.rightBumper().whileTrue(new ManualExtension(extension, -.3));
        //Player1.x().whileTrue(new PivotCommand(pivot, Constants.Positions.L4PivotPosition));
        //Player1.y().whileTrue(new ExtensionCommand(extension, 2.65));
        //Player1.a().whileTrue(new ManualExtension(extension, .5));
        //Player1.start().onTrue(new WristCommand(Wrist, 35));
        
        
        Player1.rightBumper().whileTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(1.3) 
                .withVelocityY(centerWithAprilTag()) 
                .withRotationalRate(0)
            ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),

            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(-.51) 
                .withRotationalRate(0)

            ).withTimeout(.6),
            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(0) 
                .withRotationalRate(0)

            ).until(() -> true)
        ));

        Player1.leftBumper().whileTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(1.3) 
                .withVelocityY(centerWithAprilTag()) 
                .withRotationalRate(0)
            ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),

            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(.5) 
                .withRotationalRate(0)

            ).withTimeout(.6),
            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(0) 
                .withRotationalRate(0)

            ).until(() -> true)
        ));


        /*Player1.povDown().whileTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(1.3) 
                .withVelocityY(centerWithAprilTag()) 
                .withRotationalRate(0)
            ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),

            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(-.5) 
                .withRotationalRate(0)

            ).withTimeout(.6),
            drivetrain.applyRequest(() -> driveRobotCentric

                .withVelocityX(0) 
                .withVelocityY(0) 
                .withRotationalRate(0)

            ).until(() -> true)
        ));*/

        Player1.povDown().onTrue(
            drivetrain.applyRequest(()-> driveRobotCentric
                .withVelocityX(-2.5)
                .withVelocityY(0)
                .withRotationalRate(0)
            ).withTimeout(0.45)
        );

        Player1.rightStick().whileTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
            .withVelocityX(0.8) 
            .withVelocityY(getTargetTx(true)) 
            .withRotationalRate(0)
            ).until(() -> LimelightHelpers.getTA("limelight-cg") == 0),
            drivetrain.applyRequest(() -> driveRobotCentric
            .withVelocityX(0.4) 
            .withVelocityY(0) 
            .withRotationalRate(0)
            ).withTimeout(.5)
        ));

        /*Player1.b().whileTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(0.8) 
                .withVelocityY(getTargetTx(true)) 
                .withRotationalRate(0)
            ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),

            drivetrain.applyRequest(() -> driveRobotCentric
               .withVelocityX(0.8)
               .withVelocityY(0)
               .withRotationalRate(0)
            ).withTimeout(.3),
            drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(0) 
                .withVelocityY(0) 
                .withRotationalRate(0)
                
            ).until(() -> true),
            new SequentialCommandGroup(
           new PivotCommand(pivot, Constants.Positions.L3PivotPosition),
           new WristCommand(Wrist, Constants.Positions.L3WristPosition),
           new AutoOuttakeCommand(intake)
               )));*/
        
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return null;
  
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

        private AutoRoutine LimelightMiddleAuto(){
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

            AutoRoutine routine = autoFactory.newRoutine("LimelightMid");
            AutoTrajectory LimelightMiddle1 = routine.trajectory("LimelightMiddle1");
            AutoTrajectory LimelightMiddle2 = routine.trajectory("LimelightMiddle2");
            AutoTrajectory LimelightMiddle3 = routine.trajectory("LimelightMiddle3");
            AutoTrajectory LowBallTest2 = routine.trajectory("LowBallTest2");

            routine.active().onTrue(
            Commands.sequence(
                LimelightMiddle1.resetOdometry(),
                LimelightMiddle1.cmd()
            )
        );

    
            LimelightMiddle1.done().onTrue(new SequentialCommandGroup(
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 8),
   
                drivetrain.applyRequest(() -> driveRobotCentric
                .withVelocityX(.45)
                .withVelocityY(.2)
                .withRotationalRate(0)
                ).withTimeout(.5),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
                  new ParallelCommandGroup(
                        new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                        new ParallelCommandGroup(
                            new WaitCommand(.5),
                            new WristCommand(Wrist, Constants.Positions.L4WristPosition),
                            new SequentialCommandGroup(
                                new WaitCommand(1),
                                new AutoOuttakeCommand(intake).withTimeout(2),
                                LimelightMiddle2.cmd()
                       )
                   )
               )
            )
        ));
        LimelightMiddle2.done().onTrue(
            LimelightMiddle3.cmd()
        );
        LimelightMiddle3.atTime("Reset").onTrue(new SequentialCommandGroup(
            new ExtensionCommand(extension, Constants.Positions.bringExtensionDown),
            new WaitCommand(1)
        ));
        LimelightMiddle3.atTime("LowBall").onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.LowBallPivot),
            new WristCommand(Wrist, Constants.Positions.LowBallWrist),
            new WaitCommand(1)
        ));


        LimelightMiddle3.done().onTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 5),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
            new ManualIntake(intake, 1 ).withTimeout(1),
            new WaitCommand(.5),
            LowBallTest2.cmd()
        ));
            LowBallTest2.atTime("Barge").onTrue(new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new PivotCommand(pivot, Constants.Positions.BargePivot),
                  new ParallelCommandGroup(
                        new ExtensionCommand(extension, Constants.Positions.BargeExtenstion),
                        new ParallelCommandGroup(
                            new WaitCommand(.5),
                            new WristCommand(Wrist, Constants.Positions.AutoBargeWrist)
                            
            ))));

            LowBallTest2.done().onTrue(new SequentialCommandGroup(
                new WaitCommand(3),
                new ManualIntake(intake, -1).withTimeout(1.5)
            ));
    
        
        

            return routine;


    }
    private AutoRoutine BottomLimelightAuto(){
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

        AutoRoutine routine = autoFactory.newRoutine("BottomLimelight");
        AutoTrajectory BottomLimelight = routine.trajectory("BottomLimelight");
        AutoTrajectory BottomLimelight2 = routine.trajectory("BottomLimelight2");
        AutoTrajectory BottomLimelight3 = routine.trajectory("BottomLimelight3");
        AutoTrajectory BottomLimelight4 = routine.trajectory("BottomLimelight4");
        AutoTrajectory BottomLimelight5 = routine.trajectory("BottomLimelight5");


        routine.active().onTrue(
            Commands.sequence(
                BottomLimelight.resetOdometry(),
                BottomLimelight.cmd()
            )
        );
        BottomLimelight.done().onTrue(new SequentialCommandGroup(
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(getTargetTx(true)) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 6),
   
                drivetrain.applyRequest(() -> driveRobotCentric
                   .withVelocityX(1.15)
                   .withVelocityY(0.22)
                   .withRotationalRate(0)
                ).withTimeout(.3),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
                  new ParallelCommandGroup(
                        new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                        new ParallelCommandGroup(
                            new WaitCommand(.5),
                            new WristCommand(Wrist, Constants.Positions.L4WristPosition),
                            new SequentialCommandGroup(
                                new WaitCommand(1.5),
                                new AutoOuttakeCommand(intake).withTimeout(2),
                                BottomLimelight2.cmd()
                       )
                   )
               )
            )
        ));
        BottomLimelight2.done().onTrue(new ParallelCommandGroup(
           new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist),
           new ExtensionCommand(extension, Constants.Positions.AutoL4Extenston),
            BottomLimelight3.cmd()
        ));
        

        BottomLimelight3.atTime("HumanPlayer").onTrue((new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot)
        ));

        BottomLimelight3.atTime("Intake").onTrue(Commands.sequence(
            Commands.waitSeconds(3),
            new AutoIntakeCommand(intake)
        ));


        BottomLimelight3.done().onTrue(Commands.sequence(
            new AutoIntakeCommand(intake),
            new WaitCommand(.5),
            BottomLimelight4.cmd()
        ));

        BottomLimelight4.done().onTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(getTargetTx(true)) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 6),
   
                drivetrain.applyRequest(() -> driveRobotCentric
                   .withVelocityX(1.15)
                   .withVelocityY(0.22)
                   .withRotationalRate(0)
                ).withTimeout(.3),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
                  new ParallelCommandGroup(
                        new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                        new ParallelCommandGroup(
                            new WaitCommand(.5),
                            new WristCommand(Wrist, Constants.Positions.L4WristPosition),
                            new SequentialCommandGroup(
                                new WaitCommand(1.5),
                                new AutoOuttakeCommand(intake).withTimeout(2),
                                BottomLimelight5.cmd()
                       )
                   )
               )
            )
        ));

        BottomLimelight5.done().onTrue(new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist));
            

            return routine;
    }
    private AutoRoutine LowBallAuto(){
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

        AutoRoutine routine = autoFactory.newRoutine("LowBallTest");
        AutoTrajectory LowBallTest = routine.trajectory("LowBallTest");
        AutoTrajectory LowBallTest2 = routine.trajectory("LowBallTest2");

        routine.active().onTrue(
            Commands.sequence(
                LowBallTest.resetOdometry(),
                LowBallTest.cmd()
            )
        );

        LowBallTest.atTime("LowBall").onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.LowBallPivot),
            new WristCommand(Wrist, Constants.Positions.LowBallWrist)
        ));

        LowBallTest.done().onTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 5),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
            new ManualIntake(intake, 1 ).withTimeout(.5),
            new WaitCommand(1),
            LowBallTest2.cmd()
        ));

        LowBallTest2.done().onTrue(new SequentialCommandGroup(
            new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot),
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist)
        ));

        return routine;
    }
    private AutoRoutine NewLimelightAUTO(){
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

        AutoRoutine routine = autoFactory.newRoutine("NewLimelightAUTOMID");
        AutoTrajectory NewLimelight1 = routine.trajectory("NewLimelight1");
        //unused AutoTrajectory NewLimelight2 = routine.trajectory("NewLimelight2");
        AutoTrajectory NewLimelight3 = routine.trajectory("NewLimelight3");
        //unused AutoTrajectory NewLimelight4 = routine.trajectory("NewLimelight4");
        //unused AutoTrajectory NewLimelight5 = routine.trajectory("NewLimelight5"); 

        routine.active().onTrue(
            Commands.sequence(
                NewLimelight1.resetOdometry(),
                NewLimelight1.cmd()
            )
        );

        NewLimelight1.done().onTrue(new SequentialCommandGroup(
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(1) 
                    .withVelocityY(centerWithAprilTag()) 
                    .withRotationalRate(0)
                ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),
   
                drivetrain.applyRequest(() -> driveRobotCentric
                   .withVelocityX(0)
                   .withVelocityY(-.6)
                   .withRotationalRate(0)
                ).withTimeout(.65),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
   
                ).until(() -> true),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new PivotCommand(pivot, Constants.Positions.L4PivotPosition),
                  new ParallelCommandGroup(
                        new ExtensionCommand(extension, Constants.Positions.L4ExtensionPosition),
                        new ParallelCommandGroup(
                            new WaitCommand(.5),
                            new WristCommand(Wrist, Constants.Positions.L4WristPosition),
                            new SequentialCommandGroup(
                                new WaitCommand(2),
                                new AutoOuttakeCommand(intake).withTimeout(2),
                                new SequentialCommandGroup(
                                drivetrain.applyRequest(()-> driveRobotCentric
                                    .withVelocityX(-2)
                                    .withVelocityY(0)
                                    .withRotationalRate(0)
                                ).withTimeout(0.7),
                                drivetrain.applyRequest(() -> driveRobotCentric
                                .withVelocityX(0)
                                .withVelocityY(.5)
                                .withRotationalRate(0)
                                ).withTimeout(.65),
                                drivetrain.applyRequest(() -> driveRobotCentric
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0)
                                ).withTimeout(.1),
                                new WaitCommand(2),
                                NewLimelight3.cmd()
                                )
                            )
                        )
                    )
            
            )
        ));

        NewLimelight3.done().onTrue(new SequentialCommandGroup(
            new ExtensionCommand(extension, Constants.Positions.bringExtensionDown).withTimeout(0.6),
            new PivotCommand(pivot, Constants.Positions.HumanPlayerPivot),
            new WristCommand(Wrist, Constants.Positions.HumanPlayerWrist)
            /*new SequentialCommandGroup(
                new WaitCommand(2),
                drivetrain.applyRequest(() -> driveRobotCentric
                        .withVelocityX(.1) 
                        .withVelocityY(centerWithAprilTag()) 
                        .withRotationalRate(0)
                    ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10) 
            )*/
            //NewLimelight5.cmd()   
        ));

        /*NewLimelight5.done().onTrue(new SequentialCommandGroup(
            new WaitCommand(1),
            
        ));*/
        /*NewLimelight3.done().onTrue(new SequentialCommandGroup(
            drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0.8) 
                    .withVelocityY(centerWithAprilTag()) 
                    .withRotationalRate(0)
            ).until(() -> !LimelightHelpers.getTV("limelight-cg") || LimelightHelpers.getTA("limelight-cg") > 10),
                drivetrain.applyRequest(() -> driveRobotCentric
                    .withVelocityX(0) 
                    .withVelocityY(0) 
                    .withRotationalRate(0)
            
                ).until(()->true),
                new ManualIntake(intake, 1 ).withTimeout(1),
                new WaitCommand(.5),
                NewLimelight4.cmd()
            )
        );*/

        
        

        return routine;
        
    }
}
