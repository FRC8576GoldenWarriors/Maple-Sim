package frc.robot.Subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimField extends SubsystemBase {
  private StructArrayPublisher<Pose3d> coralPosesStruct;
  private StructArrayPublisher<Pose3d> algaePosesStruct;

  private Pose3d[] algaePoses;
  

  public SimField() {

    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
        // We must specify a heading since the coral is a tube
        new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
    coralPosesStruct = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Coral_Poses", Pose3d.struct)
        .publish();

    algaePosesStruct = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Algae_Poses", Pose3d.struct)
        .publish();

    SimulatedArena.getInstance().resetFieldForAuto();
    SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));

  }

  public void periodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    // // // Get the positions of the notes (both on the field and in the air)
    // Pose3d[] notesPoses = SimulatedArena.getInstance()
    // .getGamePiecesArrayByType("Note");
    // // Publish to telemetry using AdvantageKit
    // Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);
    Pose3d[] coralPoses = SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Coral");
    coralPosesStruct.accept(coralPoses);

    algaePoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
    algaePosesStruct.accept(algaePoses);
  }


  public Pose3d[] getAlgaePosesStruct() {
    return algaePoses;
  } 
  
}
