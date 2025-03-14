package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.ScoringConstants.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class SetElevatorPoseV2 extends SequentialCommandGroup{
    public SetElevatorPoseV2(Elevator elevator, Wrist wrist, ElevatorPose pose){
        if(pose.armAngle() > safeZoneMaxAngle){
            if(pose.elevatorHeight() > safeZoneMaxHeight){
                addCommands(Commands.print("Unable to reach this pose! (unsafe)"));
                return;
            }
            addCommands(
                elevator.setSetpointCommand(pose.elevatorHeight()),
                wrist.setSetpointCommand(safeZoneMaxAngle),
                Commands.waitUntil(()-> elevator.isSafe()),
                wrist.setSetpointCommand(pose.armAngle())
            );
        }
        else if(pose.elevatorHeight() > safeZoneMaxHeight){
            addCommands(
                wrist.setSetpointCommand(pose.armAngle()),
                Commands.waitUntil(() -> wrist.isSafe()),
                elevator.setSetpointCommand(pose.elevatorHeight())
            );
        }
        else{
            addCommands(
                wrist.setSetpointCommand(pose.armAngle()),
                elevator.setSetpointCommand(pose.elevatorHeight())
            );
        }
        addCommands(Commands.waitUntil(() -> (elevator.isAtSetpoint() && wrist.isAtSetpoint())));
    }
}
