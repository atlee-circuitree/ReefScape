package frc.robot.generated;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon {
    private Pigeon2 m_pigeon;

    public Pigeon(int id) {
        m_pigeon = new Pigeon2(id, "1599-B");
    }

    public double getAngle() {
        return m_pigeon.getAngle();
    }
}
