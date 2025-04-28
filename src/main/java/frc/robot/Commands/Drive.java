package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Drive extends Command {
    private final SwerveDrive drive = RobotContainer.simSwerve;

    public Drive(){
        addRequirements(drive);
    }

    public void execute(){
        drive.drive(
        -RobotContainer.driverController.getLeftY()
            * Math.abs(RobotContainer.driverController.getLeftY())
            * Constants.SwerveConstants.xCoeffecient,// 2.25,
        -RobotContainer.driverController.getLeftX()
            * Math.abs(RobotContainer.driverController.getLeftX())
            * Constants.SwerveConstants.yCoefficient, // 2.25
        -RobotContainer.driverController.getRightX()
            * Math.abs(RobotContainer.driverController.getRightX())
            * Constants.SwerveConstants.rotCoefficinet, // 1.75
        true, // !RobotContainer.driverController.getHID().getRawButton(XboxController.Button.kB.value)
        true);
    }
}
