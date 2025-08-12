package frc.robot.Subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.TagMap.Face;
import frc.robot.Subsystems.TagMap.Tags;

public class MapleSimSwerve extends SubsystemBase implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;
    //public static TagMap map = new TagMap(AprilTagFields.k2025ReefscapeAndyMark, Tags.REEF);

    private StructPublisher<Pose2d> robotPose;
    private StructPublisher<Pose2d> actualPose;
    private StructPublisher<Pose2d> closestPose;

    private Supplier<Pose2d> pose2dForPathFinding;

    RobotConfig config;

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

      closestPose = NetworkTableInstance.getDefault()
      .getStructTopic("Closest Apriltag Pose to Robot Pose", Pose2d.struct)
      .publish();

        try{
            this.config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getFieldPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting
            // pose)
            this::getSimulatedRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) ->
                driveRobotRelative(
                    speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
            // Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                new PIDConstants(2.6, 0.2, 0.1),
                new PIDConstants(5.5, 0, 0.001)),
            this.config, // The robot configuration
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            //   return alliance.get() == DriverStation.Alliance.Red;
            // }
            return false;
            } // Reference to this subsystem to set requirements
            );

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

    public ChassisSpeeds getSimulatedRobotRelativeSpeeds() {
        return simulatedDrive.getActualSpeedsRobotRelative();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        simulatedDrive.runChassisSpeeds(speeds, new Translation2d(0, 0), false, false);
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
        closestPose.accept(RobotContainer.map.getClosestTagPoseToMoveTo(0.25, Face.FrontSide, this.getFieldPose()));
    }

    public AbstractDriveTrainSimulation getDriveTrain(){
        return simulatedDrive.getDriveTrainSimulation();
    }
}
