// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class Constants {

    public static PIDController AutoDrivePID = new PIDController(4, 0.01, 0);
    public static PIDController AutoTurnPID = new PIDController(5, 0, 0);

    public class CAN_IDs{
        public static final int FLDrive = 1;
        public static final int FLTurn = 2;
        public static final int FRDrive = 3;
        public static final int FRTurn = 4;
        public static final int BLDrive = 5;
        public static final int BLTurn = 6;
        public static final int BRDrive = 7;
        public static final int BRTurn = 8;
        public static final int pivotLeft = 9;
        public static final int pivotRight = 10;
        public static final int extensionLeft = 11;
        public static final int extensionRight = 12;
        public static final int IntakeMotorID = 13;
        public static final int wrist = 14;
        public static final int canivore_a = 15;
        public static final int canivore_b = 16;
        public static final int pigeon = 17;
        public static final int climb = 18;
    }

    public class Channels {
        public static final int WristChannel = 1;
        public static final int PivotChannel = 2;
        public static final int EncoderChannel = 5;
        public static final int pivotEncoderChannel = 4;
        public static final int armExtensionEncoderChannel = 6;
        public static final int climbEncoderChannel = 7;
        
        
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
    }
}

