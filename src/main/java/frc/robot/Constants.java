// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

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
        public static final double maxExtend = 900;
        public static final double minExtend = 0;
        public static final double wristThreshold = -1;
        public static final double upperWristThreshold = 265.0;
        public static final double armThreshold = 0.5;
        public static final double pivotThreshold = 45.0;
        public static final double upperPivotThreshold = 100.0;
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


        public static final double wristEncoderOffset = 0.396; // in rotations
        public static final double pivotEncoderOffsetRev = 0.514000667;
        public static final double armEncoderOffset = 0.49; // in rotations

        public static final double outtakeVelocity = 1;
        public static final double intakeVelocity = -1;

        public static final double outtakeTime = 1;
    }

    public class Drive {
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity
        public static double AprilStrafeCoeff = 0.18;
        public static double TagOffset = 6;
    }

    public class Positions {
        public static double L1WristPosition = 182;//182
        public static double L1PivotPosition = 65;//65

        public static double L2WristPosition = 212;//212//187
        public static double L2PivotPosition = 70;//73//61

        public static double L3WristPosition = 177;//205
        public static double L3PivotPosition = 78;//91

        public static double L4WristPosition = 210;//230
        public static double L4PivotPosition = 81.5;//85
        public static double L4ExtensionPosition = 3.6;//3.4

        public static double bringExtensionDown = .25;//.25

        public static double HumanPlayerWrist = 2;//0
        public static double HumanPlayerPivot = 77;//80

        public static double PivotClimb = 98;
        public static double WristClimb = 260;
        
        public static double StartPivot = 58;
        public static double StartWrist = 0.5;

        public static double LowBallWrist = 201;
        public static double LowBallPivot = 71;

        public static double HighBallWrist = 170;
        public static double HighBallPivot = 75;

        public static double BargeWrist = 80;
        public static double BargePivot = 69;

        public static double AutoL4Extenston = 0.1;
        public static double AutoL4Pivot = 0;
        public static double AutoL4Wrist = 0;

        

    
    }

}

