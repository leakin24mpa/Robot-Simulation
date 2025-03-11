// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ElevatorPose;
import frc.robot.Constants.ScoringConstants.PosePresets;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPose extends Command {
  
  private Elevator m_elevator;
  private Wrist m_wrist;
  private ElevatorPose m_goalPose;

  private boolean elevatorHasGoal;
  private boolean wristHasGoal;

  /** Creates a new SetElevatorPose. */
  public SetElevatorPose(Elevator elevator, Wrist wrist, ElevatorPose pose) {
    addRequirements(elevator, wrist);
    m_elevator = elevator;
    m_wrist = wrist;
    m_goalPose = pose;
  }
  public SetElevatorPose(Elevator elevator, Wrist wrist, PosePresets preset){
    addRequirements(elevator, wrist);
    m_elevator = elevator;
    m_wrist = wrist;
    m_goalPose = preset.pose;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goalPose = m_goalPose.makeSafe();
    elevatorHasGoal = false;
    wristHasGoal = false;

    
  }
  private void setElevatorSetpoint(){
    if(!elevatorHasGoal){
      m_elevator.setSetpoint(m_goalPose.elevatorHeight());
      elevatorHasGoal = true;
    }
  }
  private void setWristSetpoint(){
    if(!wristHasGoal){
      m_wrist.setSetpoint(m_goalPose.armAngle());
      wristHasGoal = true;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_goalPose.elevatorHeight() > ScoringConstants.safeZoneMaxHeight){
      setWristSetpoint();
      if(m_wrist.isSafe()){
        setElevatorSetpoint();
      }
    }
    else if(m_goalPose.armAngle() > ScoringConstants.safeZoneMaxAngle){
      setElevatorSetpoint();
      if(m_elevator.isSafe()){
        setWristSetpoint();
      }
      else{
        m_wrist.setSetpoint(ScoringConstants.safeZoneMaxAngle);
      }
    }
    else{
      setElevatorSetpoint();
      setWristSetpoint();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isAtSetpoint() && m_wrist.isAtSetpoint() && elevatorHasGoal && wristHasGoal;
  }
}
