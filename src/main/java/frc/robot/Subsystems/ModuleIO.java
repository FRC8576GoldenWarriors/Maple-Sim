package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {
    void setDriveOutputVoltage(Voltage voltage);
    void setSteerOutputVoltage(Voltage voltage);
    Rotation2d getSteerFacing();
    Angle getSteerRelativePosition();
    Angle getDriveWheelrPositioned();
}
