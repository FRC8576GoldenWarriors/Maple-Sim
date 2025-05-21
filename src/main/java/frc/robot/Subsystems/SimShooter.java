package frc.robot.Subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SimShooter implements ShooterIO {

    SimIntake intake;
    MapleSimSwerve driveSimulation;


    public SimShooter(SimIntake Intake, MapleSimSwerve driveSimulation) {
        this.intake = Intake;
        this.driveSimulation = driveSimulation;
    }

    @Override
    public void setRunning(boolean isRunning){}

    

    public void shootAlgae() {
        if (intake.getGamePiecesAmount() == 1) {
            ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET! "));
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                            driveSimulation.getPose().getTranslation(),
                            new Translation2d(),
                            driveSimulation.getMeasuredSpeeds(),
                            driveSimulation.getGyroYaw(),
                            Distance.ofBaseUnits(0.4, Meters), // initial height of the ball, in meters
                            LinearVelocity.ofBaseUnits(9.0, MetersPerSecond), // initial velocity, in m/s
                            Angle.ofBaseUnits(70, Degree)).withProjectileTrajectoryDisplayCallBack(
                                (poses) -> Logger.recordOutput("GoodShotTrajectory", poses.toArray(Pose3d[]::new)),
                                (poses) -> Logger.recordOutput("BadShotTrajectory", poses.toArray(Pose3d[]::new))));
            intake.decreaseIntakeCount();
            
        }
    }

      public void shootProcessor() {
        if (intake.getGamePiecesAmount() == 1) {
            ReefscapeAlgaeOnFly.setHitNetCallBack(
                    () -> System.out.println("ALGAE hits PROCESSOR!  Algae Count: " + intake.getGamePiecesAmount()));
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                            driveSimulation.getPose().getTranslation(),
                            new Translation2d(),
                            driveSimulation.getMeasuredSpeeds(),
                            driveSimulation.getGyroYaw(),
                            Distance.ofBaseUnits(0.4, Meters), // initial height of the ball, in meters
                            LinearVelocity.ofBaseUnits(9.0, MetersPerSecond), // initial velocity, in m/s
                            Angle.ofBaseUnits(180, Degree))); // shooter angle
            intake.decreaseIntakeCount();
        }
    }

    @Override
    public boolean isAlgaeShot() {
        return intake.getGamePiecesAmount() < 0;
    }

    @Override
    public void periodic(){
        //algaePoses = field.getAlgaePoses();
    }

}
