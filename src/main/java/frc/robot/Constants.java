// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class Constants {

    public static PIDController AutoDrivePID = new PIDController(4, 0.01, 0);
    public static PIDController AutoTurnPID = new PIDController(5, 0, 0);
    public static PIDController ExtensionPID = new PIDController(.01, 0, 0);

    public class CAN_IDs{

        public static final int pigeon = 0;
        public static final int FLEncoder = 2;
        public static final int BLEncoder = 3;
        public static final int FREncoder = 4;
        public static final int BREncoder = 5;
        public static final int FLDrive = 10;
        public static final int FLTurn = 11;
        public static final int FRDrive = 13;
        public static final int FRTurn = 14;
        public static final int BLTurn = 17;
        public static final int BLDrive = 16;
        public static final int BRDrive = 19;
        public static final int BRTurn = 20;

        public static final int IntakeMotorID = 30;
        public static final int wrist = 31;
        public static final int pivotLeft = 32;
        public static final int pivotRight = 33;
        public static final int extensionLeft = 35;
        public static final int extensionRight = 34;
        public static final int CoralCANRange = 40;
        public static final int PivotPigeon = 41;
        public static final int ExtensionCANCoder = 42;
        public static final int WristCANCoder = 43;


        public final String CANBUS_Name = "1599-B";
        public final String CANBUS_Name2 = "1599-C";
    }

    public class Channels {
        public static final int WristChannel = 1;
        public static final int PivotChannel = 2;

        public static final int EncoderChannel = 5;
        public static final int pivotEncoderChannel = 7;
        public static final int armExtensionEncoderChannel = 6;

        
        
    }


    public class Colors {
         
        // Fixed Patterns
        public static double Rainbow = -.99;
        public static double Ocean = -.95;
        public static double Lava = -.93;
        public static double Forest = -.91;


        // Color Patterns
        public static double ChaseColor = .01;
        public static double HeartbeatColor = .05;
        

        //Solid Colors 
        public static double HotPink = .57;
        public static double DarkRed = .59;
        public static double Red = .61;
        public static double RedOrange = .63;
        public static double Orange = .65;
        public static double Gold = .67;
        public static double Yellow = .69;
        public static double LawnGreen = .71;
        public static double Lime = .73;
        public static double DarkGreen = .75;
        public static double Green = .77;
        public static double BlueGreen = .79;
        public static double Aqua = .81;
        


    }

    public class Arm {
        public static final double maxExtend = 3;
        public static final double minExtend = 0;
        public static final double wristThreshold = 4;//5.8//4
        public static final double upperWristThreshold = 236; //195 for low
        public static final double armThreshold = 1;
        public static final double pivotThreshold = 5.6;
        public static final double upperPivotThreshold = 60;
        public static final double extensionThreshold = 10;
        

        public static final double pivotP = 0.15;
        public static final double pivotI = 0.0;
        public static final double pivotD = 0.0;

        public static final double wristP = .05;
        public static final double wristI = 0.0;
        public static final double wristD = 0.0;


        public static final double ArmP = 0.25;//.50
        public static final double ArmI = 0.0;
        public static final double ArmIZone = 0.0;
        public static final double ArmD = 0.0;

        public static final double wristRatio = 1;
        public static final double pivotRatio = 1;
        public static final double extensionRatio = 1;


        public static final double wristEncoderOffset = -.5 + .945; // in rotations | make 2nd value lower if 0 is negative // is ~9.7 at green thing. make wrist lower
        public static final double pivotEncoderOffsetRev = 0.9541;
        public static final double armEncoderOffset = .298; // in rotations

        public static final double outtakeVelocity = .2; //.3
        public static final double intakeVelocity = -.2;

        public static final double outtakeTime = 1;
    }

    public class Drive {
        public static boolean SpeedToggle = true; //true = fast, false = slow
        public static double Speed = 1;
        public static double maxSpeed = 1;
        public static double minSpeed = 3.5; // the value is what the speed is being divided by
        
        public static double MaxSpeed = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed 6 wads slow
        public static double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity
        public static double AprilStrafeCoeff = 0.18;
        public static double TagOffset = 6;
        //public static double SlowSpeed = TunerConstants.HalfVelocity.in(MetersPerSecond);
        
    }
/* notes
 * 
 * make l4 wrist higher, it was going over, 
 * add limelight repositioning
 * 
 */
    public class Positions {
        public static double L1WristPosition = 2;//182
        public static double L1PivotPosition = 45;//65

        public static double L2WristPosition = 216;//187
        public static double L2PivotPosition = 30;//73//61

        public static double L3WristPosition = 245;//205
        public static double L3PivotPosition = 45.3;//91
        public static double L3ExtensionPosition = 2;

        public static double L4WristPosition = 245;//230
        public static double L4PivotPosition = 45.3;//85
        public static double L4ExtensionPosition = 4.14;//2.6

        public static double bringExtensionDown = .18;//.25

        public static double HumanPlayerWrist = 9.5;//0
        public static double HumanPlayerPivot = 37;//80

        public static double PivotClimb = 53;
        public static double WristClimb = 260;
        public static double StartPivot = 20;
        public static double StartWrist = 5;

        public static double LowBallWrist = 190;//195
        public static double LowBallPivot = 23;//24

        public static double HighBallWrist = 157.5;
        public static double HighBallPivot = 30;

        public static double BargePivot = 38;
        public static double BargeExtenstion = 4.0;
        public static double AutoBargeWrist = 120;

        public static double AutoL4Extenston = 0.1;
        public static double AutoL4Pivot = 0;
        public static double AutoL4Wrist = 0;

    

        
    

    
    }
    public static final double distanceBetweenWheel = Units.inchesToMeters(29);

    public static final SwerveDriveKinematics swervePoseEstimatorKinematic = new SwerveDriveKinematics(
        new Translation2d(distanceBetweenWheel /2, distanceBetweenWheel /2),
        new Translation2d(distanceBetweenWheel /2, -distanceBetweenWheel /2),
        new Translation2d(-distanceBetweenWheel /2, distanceBetweenWheel /2),
        new Translation2d(-distanceBetweenWheel /2, -distanceBetweenWheel /2)
    );
    public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    // Wall thickness is 0.051
    public static final double[] centerOfReef = {4.487, 4.025};

    public static final List<Pose2d> blueReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(2.890, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(3.689, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(5.285, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(6.087, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(5.285, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(3.689, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};

    public static final List<Pose2d> redReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(11.466, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(12.265, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(13.861, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(14.663, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(13.861, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(12.265, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};

    public static final double leftOffset = 0.165;
    public static final double L2ScoringOffset = 0.285;
    public static final double L3ScoringOffset = 0.145;  // Was 1.55
    public static final double L4ScoringOffset = 0.315;
    public static final double topAlgaeScoringOffset = 0.23;
    public static final double bottomAlgaeScoringOffset = 0.25;

    public static final double generalScoringOffset = 0.285;

  }
}

