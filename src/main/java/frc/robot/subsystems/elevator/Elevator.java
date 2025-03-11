package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tunableControllers.TunableElevatorFeedforward;
import frc.lib.tunableControllers.TunablePID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.elevator.elevatorIO.ElevatorIO;
import frc.robot.subsystems.elevator.elevatorIO.NeoElevatorIO;
import frc.robot.subsystems.elevator.elevatorIO.SimElevatorIO;

public class Elevator extends SubsystemBase{
    private ElevatorIO elevatorIO; 
    private TunablePID pid = new TunablePID("Elevator PID", ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);//ElevatorConstants.PID;
    private TunableElevatorFeedforward feedforward = new TunableElevatorFeedforward("Elevator FF", ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);//ElevatorConstants.feedforward;
    private TrapezoidProfile profile = new TrapezoidProfile(ElevatorConstants.constraints);

    private double setpoint;
    private TrapezoidProfile.State endState;
    private TrapezoidProfile.State startState;
    private Timer timer = new Timer();



    public Elevator(){
        if(RobotBase.isSimulation()){
            elevatorIO = new SimElevatorIO();
        }
        else{
            elevatorIO = new NeoElevatorIO();
        }
        elevatorIO.resetPosition(0);
        setpoint = elevatorIO.getPosition();
        endState = new TrapezoidProfile.State(setpoint, 0);
        startState = endState;


    }

    private void moveTowardsSetpoint(){
        TrapezoidProfile.State desiredState = profile.calculate(timer.get(), startState, endState);

        
        double output = feedforward.calculate(desiredState.velocity) + pid.calculate(elevatorIO.getPosition(), desiredState.position);
        
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);

        SmartDashboard.putNumber("Elevator Desired Velocity", desiredState.velocity);
        SmartDashboard.putNumber("Elevator Desired Position", desiredState.position);

        SmartDashboard.putNumber("Elevator Actual Velocity", elevatorIO.getVelocity());
        SmartDashboard.putNumber("Elevator Actual Position", elevatorIO.getPosition());
        SmartDashboard.putBoolean("Elevator Is At Setpoint", isAtSetpoint());
        SmartDashboard.putNumber("Elevator Motor Output", output);

        if(elevatorIO.isAtUpperBound() && output > 0){
            output = 0;
            elevatorIO.resetPosition(ElevatorConstants.maxPosition);
        }
        if(elevatorIO.isAtLowerBound() && output < 0){
            output = 0;
            elevatorIO.resetPosition(ElevatorConstants.minPosition);
        }

        elevatorIO.setSpeed(output);
    }
    public double getPosition(){
        return elevatorIO.getPosition();
    }
    public boolean isAtSetpoint(){
        return Math.abs(setpoint - elevatorIO.getPosition()) < 0.01;
    }
    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
        this.endState = new TrapezoidProfile.State(setpoint, 0);
        this.startState = elevatorIO.getState();
        timer.restart();
    }
    public boolean isSafe(){
        return getPosition() <= ScoringConstants.safeZoneMaxHeight;
    }
    public Command setSetpointCommand(double setpoint){
        return runOnce(() -> {
            setSetpoint(setpoint);
        });
    }
    
    @Override
    public void periodic(){
        moveTowardsSetpoint();
        elevatorIO.update();
        pid.refresh();
        feedforward.refresh();
    }
}
