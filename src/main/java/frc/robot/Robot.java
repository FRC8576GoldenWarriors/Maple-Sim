// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  
  private RobotContainer m_robotContainer;
  private StructArrayPublisher<Pose3d> coralPoses;
  private StructArrayPublisher<Pose3d> algaePoses;
  @Override
  public void robotInit() {
    SimulatedArena.getInstance();
    SimulatedArena.overrideInstance(new Arena2025Reefscape());


    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
coralPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Coral_Poses", Pose3d.struct)
      .publish();

      algaePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Algae_Poses", Pose3d.struct)
      .publish();

      SimulatedArena.getInstance().resetFieldForAuto();
      SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));

   Logger.recordMetadata("Goldfish", "Goldfish"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

      
    }
    // } else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath =
    //       LogFileUtil
    //           .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   Logger.addDataReceiver(
    //       new WPILOGWriter(
    //           LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    FollowPathCommand.warmupCommand().schedule();
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void simulationPeriodic(){
    SimulatedArena.getInstance().simulationPeriodic();
//     //   // Get the positions of the notes (both on the field and in the air)
//     Pose3d[] notesPoses = SimulatedArena.getInstance()
//     .getGamePiecesArrayByType("Note");
// // Publish to telemetry using AdvantageKit
// Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);
Pose3d[] coralsPoses = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");
      coralPoses.accept(coralsPoses);

Pose3d[] algaesPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
      algaePoses.accept(algaesPoses);


  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    
    // m_drivetrain.setAllIdleMode(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_drivetrain.zeroHeading();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_drivetrain.setHeading((m_drivetrain.getHeading()+180));
    // m_drivetrain.setAllIdleMode(true);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
