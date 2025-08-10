// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  
  private RobotContainer m_robotContainer;
  private Mechanism2d superStructureMechanism = new Mechanism2d(3, 3);
  private MechanismRoot2d superStructureRootMechanism = superStructureMechanism.getRoot("Base", 2, 0);

  private MechanismLigament2d superStructure;
  private MechanismLigament2d endEffector;


  private StructArrayPublisher<Pose3d> zeroedComponenetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();
  private StructArrayPublisher<Pose3d> finalComponentPoses = NetworkTableInstance.getDefault().getStructArrayTopic("FinalComponentPoses", Pose3d.struct).publish();
  

  @Override
  public void robotInit() {
    SimulatedArena.getInstance();
    SimulatedArena.overrideInstance(new Arena2025Reefscape());
    
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();


    Logger.recordMetadata("Goldfish", "Goldfish"); // Set a metadata value

    
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

    superStructure = superStructureRootMechanism.append(new MechanismLigament2d("super structure", 1, 90, 5, new Color8Bit(Color.kOrange)));
    endEffector = superStructure.append(new MechanismLigament2d("end effector", 1, 90, 6, new Color8Bit(Color.kPurple)));

     

    


    FollowPathCommand.warmupCommand().schedule();
    m_robotContainer = new RobotContainer();
   
  }
  @Override
  public void simulationPeriodic(){ 
   

    SimulatedArena.getInstance().simulationPeriodic();


  }

  @Override
  public void robotPeriodic() {
    
    zeroedComponenetPublisher.set(new Pose3d[] {new Pose3d()});
    finalComponentPoses.set(new Pose3d[] {new Pose3d(0,0,0, new Rotation3d(0, Math.sin(Timer.getTimestamp()) - 1, 0))});

    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    SmartDashboard.putData("Mech2d", superStructureMechanism);

    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    CommandScheduler.getInstance().run();

   

  }

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
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_drivetrain.setHeading((m_drivetrain.getHeading()+180));
    // m_drivetrain.setAllIdleMode(true);
  }

  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}
