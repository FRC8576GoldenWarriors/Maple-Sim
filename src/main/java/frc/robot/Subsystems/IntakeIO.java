package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem {
    public void setRunning(boolean running);
    public boolean isAlgaeInsideIntake();
    
}
