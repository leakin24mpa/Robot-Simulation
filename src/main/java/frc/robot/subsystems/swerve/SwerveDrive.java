// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants.Mod0;
import frc.robot.subsystems.swerve.SwerveConstants.Mod1;
import frc.robot.subsystems.swerve.SwerveConstants.Mod2;
import frc.robot.subsystems.swerve.SwerveConstants.Mod3;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule[] swerveModules;
  private final Pigeon2 gyro;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //A pigeon2 gyro
    gyro = new Pigeon2(SwerveConstants.gyro_ID);
    //clear all the old settings on the gyro
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    //A list of the 4 swerve modules
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Mod0.driveMotorID, Mod0.angleMotorID, Mod0.encoderID, Mod0.angleOffset),
      new SwerveModule(1, Mod1.driveMotorID, Mod1.angleMotorID, Mod1.encoderID, Mod1.angleOffset),
      new SwerveModule(2, Mod2.driveMotorID, Mod2.angleMotorID, Mod2.encoderID, Mod2.angleOffset),
      new SwerveModule(3, Mod3.driveMotorID, Mod3.angleMotorID, Mod3.encoderID, Mod3.angleOffset),
    };
    //set the gyro heading to zero to start
    zeroGyro();
  }
  //For FRC games its sometimes useful to have this match your alliance side, so on blue it resets to 0 ,
  // but on red it resets to 180
  public void zeroGyro(){
    gyro.setYaw(0);
  }
  //get the output of the pigeon2 as a rotation2d
  public Rotation2d getYaw(){
    if(SwerveConstants.invertGyro){
      return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
    else{
      return Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble());
    }
  }
  

  public void teleopDrive(double xInput, double yInput, double rotationInput, boolean isFieldOriented){

    //create a chassisSpeeds to store the desired speed
    ChassisSpeeds desiredSpeeds;
    
    if(isFieldOriented){
      //If the inputs are field-oriented, then convert them to robot-oriented
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xInput, yInput, rotationInput, getYaw());
    }
    else{
      desiredSpeeds = new ChassisSpeeds(xInput,yInput, rotationInput);
    }
    //drive the robot with open-loop control
    driveFromChassisSpeeds(desiredSpeeds, true);
  }
  
  //this function drives the robot at a given ChassisSpeeds. it can be used with a joystick or with pathplanner to control the robot
  public void driveFromChassisSpeeds(ChassisSpeeds driveSpeeds, boolean isOpenLoop){
    //calculate the list of speeds and directions for each swerve module that will make the robot drive at the desired speed
    SwerveModuleState[] desiredStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(driveSpeeds);
    //if any motor is going too fast, scale back all the motors speeds to keep them in sync
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);


    //set each module to drive at its corresponding speed and direction
    for(SwerveModule module : swerveModules){
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);
    }
  }
}
