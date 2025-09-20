package frc.robot.generated;

import com.ctre.phoenix6.hardware.CANcoder;

public class CanCoder {
    
    private CANcoder m_CanCoder;

    public CanCoder(int id) {
        m_CanCoder = new CANcoder(id, "1599-B");

    }

    public double get() {
        
        return m_CanCoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getDistance() {
        
        return m_CanCoder.getPosition().getValueAsDouble();
    }

    public int getDeviceID() {
        return m_CanCoder.getDeviceID();
    }

}
