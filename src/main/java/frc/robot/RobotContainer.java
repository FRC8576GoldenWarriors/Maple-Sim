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

public class RobotContainer {

  public static CommandXboxController driverController = new CommandXboxController(0);
  public static MapleSimSwerve simSwerve = new MapleSimSwerve();
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public RobotContainer() {
    simSwerve.setDefaultCommand(new Drive());
    configureBindings();
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(()->simSwerve.zeroHeading(),simSwerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
