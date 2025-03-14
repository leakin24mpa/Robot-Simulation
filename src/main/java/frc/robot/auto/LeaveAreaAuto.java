package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.wrist.Wrist;

public class LeaveAreaAuto extends SequentialCommandGroup{
    public LeaveAreaAuto(SwerveDrive drive, Elevator elevator, Wrist wrist){
        addCommands(
            drive.setStartingPose(7.204,7.542,0),
            new InstantCommand(() -> drive.teleopDrive(-2.5, 0, 0, true), drive),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> drive.teleopDrive(0, 0, 0, true), drive)
        );
    }
}
