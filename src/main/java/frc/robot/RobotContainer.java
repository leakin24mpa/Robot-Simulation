// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ScoringConstants.PosePresets;
import frc.robot.auto.LeaveAreaAuto;
import frc.robot.auto.CircleDriveAuto;
import frc.robot.commands.ReefAutoAlign;
import frc.robot.commands.SetElevatorPoseV2;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
  //Xbox controller in port 0
  private final CommandXboxController xbox = new CommandXboxController(0);

  //Create a SwerveDrive
  private final SwerveDrive drive = new SwerveDrive();
  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();

  private final Mechanism2d mech = new Mechanism2d(2, 4);
  private final MechanismRoot2d root = mech.getRoot("root", 1, 0);
  private final MechanismLigament2d elevatorLigament = root.append(new MechanismLigament2d("elevator", 0.2, 90));
  private final MechanismLigament2d wristLigament = elevatorLigament.append(new MechanismLigament2d("wrist", 0.4, 0));
  
  public RobotContainer() {

    configureBindings();
    SmartDashboard.putData("Robot Mechanisms", mech);
    SmartDashboard.putData("Setpoint -> 2", elevator.setSetpointCommand(2));
    SmartDashboard.putData("Setpoint -> 0", elevator.setSetpointCommand(0));

    SmartDashboard.putData("Wrist Setpoint -> 45", wrist.setSetpointCommand(45));
    SmartDashboard.putData("Wrist Setpoint -> -45", wrist.setSetpointCommand(-45));

    for(PosePresets preset : PosePresets.values()){
      SmartDashboard.putData("Set Pose To: " + preset.toString(), new SetElevatorPoseV2(elevator, wrist, preset.pose));
    }
  }

  private void configureBindings() {
    //enable the TeleopSwerve command by default, and pass joystick inputs to the TeleopSwerve command
    drive.setDefaultCommand(new TeleopSwerve(drive, () -> -xbox.getLeftX(), () -> -xbox.getLeftY(), () -> xbox.getRightX()));
    xbox.button(1).whileTrue(new ReefAutoAlign(drive, true));
    xbox.button(2).whileTrue(new ReefAutoAlign(drive, false));
  }

  public void updateMechanism2d(){
    elevatorLigament.setLength(0.2 + elevator.getPosition());
    wristLigament.setAngle(wrist.getAngle().plus(Rotation2d.kCW_90deg));
  }
  public Command getAutonomousCommand() {
    switch(AutoConstants.getSelectedAuto()){
      case TEST_AUTO:
        return new CircleDriveAuto(drive, elevator, wrist);
      default:
        return new LeaveAreaAuto(drive, elevator, wrist);
    }
  }
}
