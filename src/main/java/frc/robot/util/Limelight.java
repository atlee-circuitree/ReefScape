package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Limelight {
    private final String name;
    private final NetworkTable limelight;
    private final PIDController pidTA;
    private final PIDController pidTX;
    private final PIDController pidRotate;

    public Limelight(String name) {
        this.name = name;
        this.limelight = NetworkTableInstance.getDefault().getTable(name);
        this.pidTA = new PIDController(0.45, 0, 0);
        this.pidTX = new PIDController(0.07, 0, 0);
        this.pidRotate = new PIDController(0.07, 0, 0);
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

    /*public double getDistanceToAprilTag() {
        if (getTV() < 1) {
            return 0;
        }

        double ta = Math.abs(getTA());

        
        if (name.equals("limelight-right")) {

            if (ta < 5) {
                return 1;
            } else if (ta > 20) {
                return .2;
            } else {
                return ta/10;
            }

        }

        if (name.equals("limelight-left")) {
            if (ta < .)
        }
        
        return 0;
    }*/

    public double getFowardRate() {
        
        if (getTV() == 0) {return 0;}

        if (getName().equals("limelight-right")) {
            if (getTA() < 2) {
                return 2;
            }
            
            return pidTA.calculate(getTA()/10, 3.3);
        } else if (getName().equals("limelight-left")) {
            if (getTA() < 1) {
                return 2;
            }
            
            return pidTA.calculate(getTA()/10, 2.05);
        }
        

        return 0;

    }

    public double getVerticalRate() {
        
        if (getTV() == 0) {return 0;}
        if (getName().equals("limelight-right")) {
            return pidTX.calculate(getTX(), 7.7);
        } else if (getName().equals("limelight-left")) {
            return pidTX.calculate(getTX(), 16);
        }

        return 0;
    }

    public double getLeftFowardRate() {

        if (getTV() == 0) {return 0;}
        
        return pidTA.calculate(getTA()/10, 2.1);

    }

    public double getRotationalRate() {
        
        if (getTV() == 0) {return 0;}
        if (getName().equals("limelight-right")) {
            return pidRotate.calculate(getTX(), 7.7);
        } else if (getName().equals("limelight-left")) {
            return pidRotate.calculate(getTX(), 16);
        }

        return 0;
    }

    public static Limelight getOptimalLimelight(Limelight limelight_1, Limelight limelight_2) {
        double tx_1 = Math.abs(limelight_1.getTX());
        double tx_2 = Math.abs(limelight_2.getTX());
        if (limelight_1.getTV() == 0) {
            return limelight_2;
        } else if (limelight_2.getTV() == 0) {
            return limelight_1;
        }

        if (limelight_1.getTV() == 1 && limelight_2.getTV() == 1) {
            if (tx_1 < tx_2) {
                return limelight_1;
            } else {
                return limelight_2;
            }
        }

        return limelight_1;
    }

    public String getName() {
        return name;
    }



}
