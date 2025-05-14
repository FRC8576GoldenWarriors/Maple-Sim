package frc.robot.Subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

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
    public void setRunning(boolean isRunning) {

    }

    public void shootAlgae() {
        if (intake.getGamePiecesAmount() == 1) {
            ReefscapeAlgaeOnFly.setHitNetCallBack(
                    () -> System.out.println("ALGAE hits NET!  Algae Count: " + intake.getGamePiecesAmount()));
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                            driveSimulation.getPose().getTranslation(),
                            new Translation2d(),
                            driveSimulation.getMeasuredSpeeds(),
                            driveSimulation.getGyroYaw(),
                            Distance.ofBaseUnits(0.4, Meters), // initial height of the ball, in meters
                            LinearVelocity.ofBaseUnits(9.0, MetersPerSecond), // initial velocity, in m/s
                            Angle.ofBaseUnits(70, Degree))); // shooter angle
            intake.decreaseIntakeCount();
        }
    }

      public void shootProcessor() {
        if (intake.getGamePiecesAmount() == 1) {
            ReefscapeAlgaeOnFly.setHitNetCallBack(
                    () -> System.out.println("ALGAE hits NET!  Algae Count: " + intake.getGamePiecesAmount()));
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
        
        
    // intake.decreaseIntakeCount();
        // .withProjectileTrajectoryDisplayCallBack(
        // (poses) -> Logger.recordOutput("successfulShotsTrajectory",
        // poses.toArray(Pose3d[]::new)),
        // (poses) -> Logger.recordOutput("missedShotsTrajectory",
        // poses.toArray(Pose3d[]::new))));

        // Translation2d robotPosition,
        // Translation2d shooterPositionOnRobot,
        // ChassisSpeeds chassisSpeeds,
        // Rotation2d shooterFacing,
        // Distance initialHeight,
        // LinearVelocity launchingSpeed,
        // Angle shooterAngle
    }

    @Override
    public boolean isAlgaeShot() {
        return intake.getGamePiecesAmount() < 0;
    }

    @Override
    public void periodic(){
        System.out.println("Algae Count: "+intake.getGamePiecesAmount());
    }

}
