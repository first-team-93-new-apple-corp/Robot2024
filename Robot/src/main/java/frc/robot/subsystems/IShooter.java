package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IShooter extends ISubsystem{
    public void intake();
    public void stop();
    public void shootAmp();
    public void shootSpeaker();
}
