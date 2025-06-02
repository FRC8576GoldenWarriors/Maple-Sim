// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.MapleSimSwerve;
import frc.robot.Subsystems.SimField;
import frc.robot.Subsystems.SimIntake;
import frc.robot.Subsystems.SimShooter;

public class RobotContainer {

  public static CommandXboxController driverController = new CommandXboxController(0);
  public static MapleSimSwerve simSwerve = new MapleSimSwerve();
  public static SimIntake intake = new SimIntake(simSwerve.getDriveTrain());
  public static SimField field = new SimField();
  public static SimShooter shooter = new SimShooter(intake, simSwerve);
  
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
    
  }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
