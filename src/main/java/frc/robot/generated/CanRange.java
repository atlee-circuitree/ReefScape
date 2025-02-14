package frc.robot.generated;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.DistanceUnit;

public class CanRange {

    private CANrange m_CANRange;
    
    public CanRange(int CanId){
        m_CANRange = new CANrange(CanId, "1599-B");
    }

    public double getDistance(DistanceUnit unit) {
        return m_CANRange.getDistance().getValue().in(unit);
    }
    
}
