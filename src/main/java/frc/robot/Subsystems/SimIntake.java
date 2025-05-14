package frc.robot.Subsystems;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimIntake implements IntakeIO {
    private final IntakeSimulation intake;
    public SimIntake(AbstractDriveTrainSimulation driveTrainSim){
        intake = IntakeSimulation.OverTheBumperIntake("Algae", driveTrainSim, Meters.of(0.1), Meters.of(0.2), 
        IntakeSimulation.IntakeSide.FRONT, 1);
    }

    @Override
    public void setRunning(boolean isRunning){
        if(isRunning)
        intake.startIntake();
        else{
        intake.stopIntake();
        intake.setGamePiecesCount(0);
    }
    }

    public int getGamePiecesAmount(){
        return intake.getGamePiecesAmount();
    }
    public void decreaseIntakeCount(){
        if(getGamePiecesAmount()>=1)
        intake.setGamePiecesCount(0);
    }

    @Override
    public boolean isAlgaeInsideIntake(){
        return getGamePiecesAmount()>0;
    }
  
    public void periodic(){
        SmartDashboard.putBoolean("Algae_In_Intake", isAlgaeInsideIntake());
    }
}
