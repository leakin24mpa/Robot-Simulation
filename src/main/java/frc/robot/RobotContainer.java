// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
  //Xbox controller in port 0
  private final XboxController xbox = new XboxController(0);
  private final CommandJoystick js = new CommandJoystick(1);

  //Create a SwerveDrive
  private final SwerveDrive drive = new SwerveDrive();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //enable the TeleopSwerve command by default, and pass joystick inputs to the TeleopSwerve command
    drive.setDefaultCommand(new TeleopSwerve(drive, () -> -xbox.getLeftX(), () -> -xbox.getLeftY(), () -> -js.getRawAxis(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
