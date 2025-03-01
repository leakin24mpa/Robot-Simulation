// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopSwerve extends Command {

  private final SwerveDrive m_Drive;
  private final DoubleSupplier m_xInput;
  private final DoubleSupplier m_yInput;
  private final DoubleSupplier m_rotationInput;

  private final SlewRateLimiter m_xRateLimiter;
  private final SlewRateLimiter m_yRateLimiter;
  private final SlewRateLimiter m_rotationRateLimiter;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveDrive drive, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotationInput) {
    /*This tells the code that the swerveDrive subsystem is 'busy' 
    while this command is running so it doesn't accidentally get
    asked to do something else by another command.*/
    addRequirements(drive);

    //save a copy of the subsystem in a variable so we can access it later
    m_Drive = drive;
    //these doublesuppliers will get the imputs from the controller and pass them to the swerve drive subsystem
    m_xInput = xInput;
    m_yInput = yInput;
    m_rotationInput = rotationInput;

    //Slew rate limiters smooth out sudden changes in the joystick inputs. This helps the robot move more smoothly and not tip over.
    m_xRateLimiter = new SlewRateLimiter(SwerveConstants.translationSlewRateLimit);
    m_yRateLimiter = new SlewRateLimiter(SwerveConstants.translationSlewRateLimit);

    m_rotationRateLimiter = new SlewRateLimiter(SwerveConstants.rotationSlewRateLimit);
    
  }

  

  //This execute() function runs repeatedly while the command is active
  @Override
  public void execute() {
    //get the values from the doublesuppliers, and apply the slew rate limiters
    double smoothedXinput = m_xRateLimiter.calculate(m_xInput.getAsDouble());
    double smoothedYinput = m_yRateLimiter.calculate(m_yInput.getAsDouble());
    double smoothedRotationinput = m_rotationRateLimiter.calculate(m_rotationInput.getAsDouble());
    

    /*the controller will give values from -1 to 1, so to convert them to meters/second, we multiply by the robot's max speed */
    double xSpeed = smoothedXinput * SwerveConstants.maxSpeed;
    double ySpeed = smoothedYinput * SwerveConstants.maxSpeed;
    double rotationSpeed = smoothedRotationinput * SwerveConstants.maxAngularVelocity;
    

    //send the speeds to the swerveDrive to make it drive. 
    m_Drive.teleopDrive(xSpeed, ySpeed, rotationSpeed, true);


  }
}
