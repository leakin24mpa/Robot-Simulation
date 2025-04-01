// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.subsystems.swerve.odometry.Pigeon2Odometry;
import frc.robot.subsystems.swerve.odometry.SimOdometry;
import frc.robot.subsystems.swerve.odometry.SwerveOdometryIO;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule[] swerveModules;
 

  private final StructArrayPublisher<SwerveModuleState> swerveDataPublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> desiredSwerveDataPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Desired Swerve States", SwerveModuleState.struct).publish();

  private final SwerveOdometryIO odometry;
  private final Field2d field;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    

    //A list of the 4 swerve modules
    swerveModules = new SwerveModule[4];
    for(int i = 0; i < 4; i++){
      ModuleData data = SwerveConstants.moduleData[i];
      swerveModules[i] = new SwerveModule(i,data);
    }
    if(RobotBase.isSimulation()){
      odometry = new SimOdometry(SwerveConstants.swerveKinematics, this::getAllModulePositions);
    }
    else{
      odometry = new Pigeon2Odometry(SwerveConstants.swerveKinematics, this::getAllModulePositions, SwerveConstants.gyro_ID);
    }

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
      odometry.setGyroHeading(Rotation2d.fromDegrees(180));
    }
    else{
      odometry.setGyroHeading(Rotation2d.fromDegrees(0));
    }
  }

  public Rotation2d getYaw(){
    return getRobotPose().getRotation();
  }
  public void setRobotPose(Pose2d pose){
    odometry.setRobotPose(pose);
  }
  public Pose2d getRobotPose(){
    return odometry.getRobotPose();
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
  
  public Command setStartingPose(double x, double y, double degrees){
    return runOnce(() -> {
      double newY = y;
      if(AutoConstants.isRightSideAuto()){
        newY = FieldConstants.FIELD_WIDTH - y;
      }
      Pose2d pose = new Pose2d(x, newY, Rotation2d.fromDegrees(degrees));
      pose = FieldConstants.flipForAlliance(pose);
      setRobotPose(pose);
    });
  }
  public Command autoDrive(String filename){
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(filename);
      if(AutoConstants.isRightSideAuto()){
        return AutoBuilder.followPath(path.mirrorPath());
      }
      else{
        return AutoBuilder.followPath(path);
      }
    }
    catch(Exception e){
      DriverStation.reportError("The path is not pathing: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }
  public Command driveToPose(Pose2d pose){
    return run(() -> {
      PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
      goalState.pose = pose;
      ChassisSpeeds drivingSpeed = AutoConstants.ppSwerveController.calculateRobotRelativeSpeeds(getRobotPose(), goalState);
      driveFromChassisSpeeds(drivingSpeed, false);
    }).until(() -> {
      Pose2d relative = pose.relativeTo(getRobotPose());
      return relative.getTranslation().getNorm() < 0.03 && relative.getRotation().getCos() > 0.95;
    });
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
    
    odometry.update();
    
    
    
    updateOdometryWithVision();
    
    field.setRobotPose(getRobotPose());
    SmartDashboard.putData("Field", field);
    for(SwerveModule module : swerveModules){
      module.update();
    }
    swerveDataPublisher.set(getStates());
    
  }
}
