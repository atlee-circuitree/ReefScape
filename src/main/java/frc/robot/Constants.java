// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

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
        public static final double wristThreshold = 0.5;
        public static final double armThreshold = 0.5;
        public static final double pivotThreshold = 0.5;
        public static final double extensionThreshold = 3;

        public static final double pivotP = 0.1;
        public static final double pivotI = 0.0;
        public static final double pivotD = 0.0;

        public static final double wristP = .5;
        public static final double wristI = .0;
        public static final double wristD = 0.0;


        /*public static final double armP = 0.1;
        public static final double armI = 0.0;
        public static final double armD = 0.0;*/

        public static final double wristRatio = 1;
        public static final double pivotRatio = 4.8;
        public static final double extensionRatio = 1;
        public static final double pivotZeroDegree = 0;

        public static final double wristEncoderOffset = 0.606; // in rotations
        public static final double pivotEncoderOffset = 45.43;
        public static final double armEncoderOffset = 0.279297; // in rotations

        public static final double outtakeVelocity = 1;
        public static final double intakeVelocity = -1;

        public static final double outtakeTime = 1;
    }

}

