package frc.robot.subsystems.elevator.elevatorIO;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface ElevatorIO {
    public double getPosition();
    public double getVelocity();
    public TrapezoidProfile.State getState();

    public boolean isAtUpperBound();
    public boolean isAtLowerBound();

    public void resetPosition(double newPosition);
    public void setSpeed(double speed);

    public void update();
}
