// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleData;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule[] swerveModules;
  private final Pigeon2 gyro;

  private final StructArrayPublisher<SwerveModuleState> swerveDataPublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> desiredSwerveDataPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Desired Swerve States", SwerveModuleState.struct).publish();

  private final SwerveDrivePoseEstimator odometry;
  private final Field2d field;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //A pigeon2 gyro
    gyro = new Pigeon2(SwerveConstants.gyro_ID);
    //clear all the old settings on the gyro
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    //A list of the 4 swerve modules
    swerveModules = new SwerveModule[4];
    for(int i = 0; i < 4; i++){
      ModuleData data = SwerveConstants.moduleData[i];
      swerveModules[i] = new SwerveModule(i,data.driveMotorID(), data.angleMotorID(), data.encoderID(), data.angleOffset());
    }

    //set the gyro heading to zero to start
    zeroGyro();
    odometry = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(), getAllModulePositions(), new Pose2d());
    field = new Field2d();

    configurePathPlanner();
  }
  
  public void configurePathPlanner(){
    AutoBuilder.configure(
      this::getRobotPose,
      this::setRobotPose,
      this::getChassisSpeeds,
      (speeds, feedforwards) -> driveFromChassisSpeeds(speeds, true),
      AutoConstants.ppSwerveController,
      AutoConstants.ppConfig,
      FieldConstants::isRedAlliance,
      this
    );
  }
  //For FRC games its sometimes useful to have this match your alliance side, so on blue it resets to 0 ,
  // but on red it resets to 180
  public void zeroGyro(){
    if(FieldConstants.isRedAlliance()){
      gyro.setYaw(180);
    }
    else{
      gyro.setYaw(0);
    }
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
  public void setRobotPose(Pose2d pose){
    gyro.setYaw(pose.getRotation().getMeasure());
    odometry.resetPosition(pose.getRotation(), getAllModulePositions(), pose);
  }
  public Pose2d getRobotPose(){
    return odometry.getEstimatedPosition();
  }

  //get an array conataining all four modules' positions
  private SwerveModulePosition[] getAllModulePositions(){
    SwerveModulePosition[] allPositions = new SwerveModulePosition[4]; //array of 4 positions
    for(SwerveModule module : swerveModules){
      allPositions[module.moduleNumber] = module.getPosition();
    }
    return allPositions;
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
  
  public Command setStartingPose(Pose2d pose){
    return runOnce(() -> setRobotPose(FieldConstants.flipForAlliance(pose)));
  }
  public Command autoDrive(String filename){
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(filename);
      return AutoBuilder.followPath(path);
    }
    catch(Exception e){
      DriverStation.reportError("The path is not pathing: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule module : swerveModules){
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }
  //this function drives the robot at a given ChassisSpeeds. it can be used with a joystick or with pathplanner to control the robot
  public void driveFromChassisSpeeds(ChassisSpeeds driveSpeeds, boolean isOpenLoop){
    //calculate the list of speeds and directions for each swerve module that will make the robot drive at the desired speed
    SwerveModuleState[] desiredStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(driveSpeeds);
    //if any motor is going too fast, scale back all the motors speeds to keep them in sync
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    desiredSwerveDataPublisher.set(desiredStates);

    //set each module to drive at its corresponding speed and direction
    for(SwerveModule module : swerveModules){
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);
    }
  }

  public ChassisSpeeds getChassisSpeeds(){
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }
  private void updateOdometryWithVision(){
    LimelightHelpers.SetRobotOrientation("limelight", getYaw().getDegrees(),0, 0, 0, 0, 0);
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(estimate == null){
      return;
    }
    if(estimate.tagCount >= 1 && estimate.rawFiducials[0].ambiguity < 0.5){
      odometry.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }
  }

  @Override
  public void periodic(){
    odometry.update(getYaw(), getAllModulePositions());
    
    updateOdometryWithVision();
    
    field.setRobotPose(getRobotPose());
    SmartDashboard.putData("Field", field);
    for(SwerveModule module : swerveModules){
      module.update();
    }
    swerveDataPublisher.set(getStates());
    
  }
}
