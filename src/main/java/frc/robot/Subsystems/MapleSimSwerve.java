package frc.robot.Subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MapleSimSwerve implements SwerveDrive{
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    private StructPublisher<Pose2d> robotPose;
    private StructPublisher<Pose2d> actualPose;
    public MapleSimSwerve(){
        final DriveTrainSimulationConfig config = 
        DriveTrainSimulationConfig.Default();

        simulatedDrive = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(config, new Pose2d(1.27,1.27,new Rotation2d())));
        
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        field2d = new Field2d();
        SmartDashboard.putData("Sim field",field2d);

        actualPose = NetworkTableInstance.getDefault()
      .getStructTopic("ActualPose", Pose2d.struct)
      .publish();

        robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("RobotPose", Pose2d.struct)
      .publish();
    }

    @Override
    public void drive(double forwardSpeed, double sideSpeed,double rotation, boolean fieldRelative, boolean isOpenLoop){
        simulatedDrive.
        runChassisSpeeds(new ChassisSpeeds(forwardSpeed,
        sideSpeed,
        rotation), 
        new Translation2d(), fieldRelative, isOpenLoop);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates){
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds(){
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw(){
        return simulatedDrive.getRawGyroAngle();
    }

    public double getGyroDegrees(){
        return simulatedDrive.getRawGyroAngle().getDegrees();
    }
    @Override
    public Pose2d getPose(){
        return simulatedDrive.getOdometryEstimatedPose();
    }

    public Pose2d getFieldPose(){
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    @Override 
    public void setPose(Pose2d pose){
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void periodic(){
        simulatedDrive.periodic();

        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
        actualPose.accept(simulatedDrive.getActualPoseInSimulationWorld());
        robotPose.accept(getPose());
    }

    public AbstractDriveTrainSimulation getDriveTrain(){
        return simulatedDrive.getDriveTrainSimulation();
    }
}
