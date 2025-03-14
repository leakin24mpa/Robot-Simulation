package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.wrist.Wrist;

public class CircleDriveAuto extends SequentialCommandGroup{
    public CircleDriveAuto(SwerveDrive drive, Elevator elevator, Wrist wrist){
        addCommands(
            drive.setStartingPose(2, 7, 180),
            drive.autoDrive("Pathy McPathFace")
        );
    }
}
