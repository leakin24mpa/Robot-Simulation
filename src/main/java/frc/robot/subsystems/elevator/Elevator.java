package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.elevatorIO.ElevatorIO;
import frc.robot.subsystems.elevator.elevatorIO.SimElevatorIO;

public class Elevator extends SubsystemBase{
    private ElevatorIO elevatorIO = new SimElevatorIO();
    private PIDController pid = ElevatorConstants.PID;
    private ElevatorFeedforward feedforward = ElevatorConstants.feedforward;
    private TrapezoidProfile profile = new TrapezoidProfile(ElevatorConstants.constraints);

    private double setpoint;
    private TrapezoidProfile.State endState;
    private TrapezoidProfile.State startState;
    private Timer timer = new Timer();

    private Mechanism2d elevatorVisual = new Mechanism2d(3,3);
    private MechanismRoot2d root = elevatorVisual.getRoot("root", 2,0);
    private MechanismLigament2d elevatorLigament = root.append(new MechanismLigament2d("elevator", 1, 90));

    public Elevator(){
        elevatorIO.resetPosition(0);
        setpoint = elevatorIO.getPosition();
        endState = new TrapezoidProfile.State(setpoint, 0);
        startState = endState;

        SmartDashboard.putData("Elevator", elevatorVisual);

    }

    private void moveTowardsSetpoint(){
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        
        TrapezoidProfile.State desiredState = profile.calculate(timer.get(), startState, endState);

        SmartDashboard.putNumber("Elevator Desired Velocity", desiredState.velocity);
        SmartDashboard.putNumber("Elevator Desired Position", desiredState.position);

        SmartDashboard.putNumber("Elevator Actual Velocity", elevatorIO.getVelocity());
        SmartDashboard.putNumber("Elevator Actual Position", elevatorIO.getPosition());

        

        double output = feedforward.calculate(desiredState.velocity) + pid.calculate(elevatorIO.getPosition(), desiredState.position);

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
    public boolean isAtSetpoint(){
        return Math.abs(setpoint - elevatorIO.getPosition()) < 0.01;
    }
    public Command setSetpoint(double setpoint){
        return runOnce(() -> {
            this.setpoint = setpoint;
            this.endState = new TrapezoidProfile.State(setpoint, 0);
            this.startState = elevatorIO.getState();
            timer.restart();
        });
    }
    @Override
    public void periodic(){
        moveTowardsSetpoint();
        elevatorIO.update();
        elevatorLigament.setLength(1 + elevatorIO.getPosition());
        
    }
}
