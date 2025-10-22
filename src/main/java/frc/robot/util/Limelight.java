package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final String name;
    private final NetworkTable limelight;
    private final PIDController pidTA;
    private final PIDController pidTX;

    public Limelight(String name) {
        this.name = name;
        this.limelight = NetworkTableInstance.getDefault().getTable(name);
        this.pidTA = new PIDController(0.1, 0, 0);
        this.pidTX = new PIDController(0.145, 0, 0.01);
    } 

    public double getTX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getTA() {
        return limelight.getEntry("ta").getDouble(0);
    }

    public double getTV() {
        return limelight.getEntry("tv").getDouble(0);
    }

    public double getDistanceToAprilTag() {
        if (getTV() < 1) {
            return -1;
        }
        
        /*double[] botpose = limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        
        if (botpose.length < 1) {
            return -1;
        }
        
        return botpose[0]; // Just the X (forward) component*/

        double ta = getTA();

        return 22-ta;
    }

    public double getFowardaRate() {
        
        return pidTA.calculate(getDistanceToAprilTag(), 1);

    }

    public double getRotationalRate() {

        return pidTX.calculate(getTX(), 0);

    }



}
