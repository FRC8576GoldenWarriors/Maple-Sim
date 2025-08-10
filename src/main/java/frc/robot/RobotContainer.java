// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.MapleSimSwerve;
import frc.robot.Subsystems.SimField;
import frc.robot.Subsystems.SimIntake;
import frc.robot.Subsystems.SimShooter;
import frc.robot.Subsystems.TagMap;
import frc.robot.Subsystems.TagMap.Face;
import frc.robot.Subsystems.TagMap.Tags;

public class RobotContainer {

  public static CommandXboxController driverController = new CommandXboxController(0);
  public static MapleSimSwerve simSwerve = new MapleSimSwerve();
  public static SimIntake intake = new SimIntake(simSwerve.getDriveTrain());
  public static SimField field = new SimField();
  public static SimShooter shooter = new SimShooter(intake, simSwerve);
  public static TagMap map = new TagMap(AprilTagFields.k2025ReefscapeAndyMark, Tags.ALL);
  
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public RobotContainer() {
    simSwerve.setDefaultCommand(new Drive());
    configureBindings();
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(()->simSwerve.zeroHeading(),simSwerve));

    driverController.a().whileTrue(new InstantCommand(()->intake.setRunning(true)));
    driverController.x().whileTrue(new InstantCommand(()->intake.setRunning(false)));
    driverController.leftBumper().onTrue(new InstantCommand(() -> shooter.shootAlgae()));
    driverController.rightBumper().onTrue(new InstantCommand(() -> shooter.shootProcessor()));

    driverController.b().whileTrue(map.getPathFindCommand(0.25, Face.BackSide));

    driverController.povUp().whileTrue(map.getPathFindCommand(18, 0.25, Face.BackSide));
    driverController.povLeft().whileTrue(map.getPathFindCommand(12));
    driverController.povDown().whileTrue(map.getPathFindCommand(16));
    driverController.povRight().whileTrue(map.getPathFindCommand(15));
    
  }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
