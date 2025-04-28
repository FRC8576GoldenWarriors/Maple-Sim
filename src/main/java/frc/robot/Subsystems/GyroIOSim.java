package frc.robot.Subsystems;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO {
    public final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation){
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public AngularVelocity getAngularVelocity(){
        return gyroSimulation.getMeasuredAngularVelocity();
    }
    @Override
    public Rotation2d getRotation2d(){
        return gyroSimulation.getGyroReading();
    }
}
