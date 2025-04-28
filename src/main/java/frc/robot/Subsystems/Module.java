package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class Module implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;

    private final SimulatedMotorController.GenericMotorController driveMotor;

    private final SimulatedMotorController.GenericMotorController turnMotor;

    public Module(SwerveModuleSimulation moduleSimulation){
        this.moduleSimulation = moduleSimulation;

        driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(60));

        turnMotor = moduleSimulation.useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(20));
    }

    @Override
    public void setDriveOutputVoltage(Voltage voltage){
        driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setSteerOutputVoltage(Voltage voltage){
        turnMotor.requestVoltage(voltage);
    }

    @Override
    public Rotation2d getSteerFacing(){
        return moduleSimulation.getSteerAbsoluteFacing();
    }

    @Override
    public Angle getSteerRelativePosition(){
        return moduleSimulation.getSteerRelativeEncoderPosition().divide(moduleSimulation.getSteerMotorConfigs().gearing);
    }

    @Override
    public Angle getDriveWheelrPositioned(){
        return moduleSimulation.getDriveWheelFinalPosition();
    }
}
