package frc.robot.generated;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {
    private Pigeon2 m_pigeon;

    public Pigeon(int id) {
        m_pigeon = new Pigeon2(id, "1599-B");
    }

    public double getAngle() {
        return -m_pigeon.getAccumGyroZ().getValueAsDouble(); //91.1 43.37
    }
    public Rotation2d getHeading(){

        return new Rotation2d(m_pigeon.getYaw().getValue());
    }
}
