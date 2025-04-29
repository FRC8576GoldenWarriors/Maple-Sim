package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.littletonrobotics.junction.AutoLog;


public interface SwerveDriveIO extends Subsystem {

    @AutoLog
    public static class SwerveDriveInputs{
        public Pose2d actualPose = new Pose2d();
        public Pose2d odomPose = new Pose2d();
        public double gyroAngle = 0;
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    }

    public default void updateInputs(SwerveDriveInputs inputs){}
    // void drive(double forwardSpeed,double sideSpeed, double rotation, boolean fieldRelative, boolean isOpenLoop);


    // void setModuleStates(SwerveModuleState[] desiredStates);
    
    // ChassisSpeeds getMeasuredSpeeds();

    // Rotation2d getGyroYaw();

    // Pose2d getPose();

    // void setPose(Pose2d pose);

    // default Rotation2d getHeading(){
    //     return getPose().getRotation();
    // }

    // default void setHeading(Rotation2d heading){
    //     setPose(new Pose2d(getPose().getTranslation(),heading));
    // }
    
    // default void zeroHeading(){
    //     setHeading(new Rotation2d());
    // }
}
