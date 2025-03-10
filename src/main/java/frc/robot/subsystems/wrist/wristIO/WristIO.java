package frc.robot.subsystems.wrist.wristIO;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    public void setSpeed(double speed);
    public void update();
    
    public Rotation2d getAngle();
    public Rotation2d getAngularVelocity();
}
