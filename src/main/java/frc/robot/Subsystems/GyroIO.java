package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
    Rotation2d getRotation2d();
    AngularVelocity getAngularVelocity();
}
