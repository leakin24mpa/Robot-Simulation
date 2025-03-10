package frc.robot.subsystems.elevator.elevatorIO;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class SimElevatorIO implements ElevatorIO{

    private DCMotor elevatorMotor = DCMotor.getNEO(1);
    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            elevatorMotor,
            6,
            ElevatorConstants.spoolRadius,
            ElevatorConstants.gearRatio
        ), null, ElevatorConstants.minPosition,ElevatorConstants.maxPosition, true, 0);

    public SimElevatorIO(){

    }
    public double getPosition() {
        return sim.getPositionMeters();
    }
    public double getVelocity(){
        return sim.getVelocityMetersPerSecond();
    }
    public TrapezoidProfile.State getState(){
        return new TrapezoidProfile.State(getPosition(), getVelocity());
    }
    public boolean isAtUpperBound(){
        return sim.hasHitUpperLimit();
    }
    public boolean isAtLowerBound(){
        return sim.hasHitLowerLimit();
    }
    public void resetPosition(double newPosition) {
        sim.setState(newPosition,0);
    }

    public void setSpeed(double speed) {
        sim.setInput(Math.min(Math.max(speed, -1), 1) * 12.0);
    }

    public void update() {
        sim.update(0.02);
    }
    
}
