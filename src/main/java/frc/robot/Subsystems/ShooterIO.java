package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem{
    public void setRunning(boolean isRunning);
    public boolean isAlgaeShot();
}
