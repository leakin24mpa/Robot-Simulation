// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.SwerveConstants.ModuleData;
import frc.robot.subsystems.swerve.moduleIO.NeoModuleIO;
import frc.robot.subsystems.swerve.moduleIO.SimModuleIO;
import frc.robot.subsystems.swerve.moduleIO.SwerveModuleIO;

/** Add your docs here. */
public class SwerveModule {

    private final SwerveModuleIO moduleIO;
    public final int moduleNumber;
    
    public SwerveModule(int moduleNumber, ModuleData data){
        //identifies which of the four modules this is
        this.moduleNumber = moduleNumber;
        if(RobotBase.isSimulation()){
            moduleIO = new SimModuleIO();
        }
        else{
            moduleIO = new NeoModuleIO(data);
        }
        
    }
    
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){

        //find how far off the wheel's actual direction is from its target direction
        double difference = desiredState.angle.getDegrees() - currentAngle.getDegrees();
        //find the least possible amount to turn by that will still get you to the correct angle
        double turnAmount = Math.IEEEremainder(difference, 360);
        
        double speed = desiredState.speedMetersPerSecond;

        
        //if we're more than 90 degrees off from the goal direction, aim in the reverse direction and drive the wheel backwards
        if(turnAmount > 90){
            turnAmount -= 180;
            speed *= -1;
        }
        if(turnAmount < -90){
            turnAmount += 180;
            speed *= -1;
        }
        double direction = currentAngle.getDegrees() + turnAmount;
        //return the updated speed and directionwa
        return new SwerveModuleState(speed, Rotation2d.fromDegrees(direction));
    }
    /** Set the goal state of the module to a speed and direction */
    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //optimize the desired state to get the module to its goal as fast as possible
        SwerveModuleState optimizedState = optimize(desiredState, getAngle());
        
        double cosineComp = optimizedState.angle.minus(getAngle()).getCos();

        moduleIO.setAngle(optimizedState.angle);
        moduleIO.setSpeed(optimizedState.speedMetersPerSecond * cosineComp, isOpenLoop);
        return optimizedState;
    }
   
    //get the angle from the main encoder
    public Rotation2d getAngle(){
        return moduleIO.getAngle();
    }
    public SwerveModuleState getState(){
        return moduleIO.getState();
    }
    public SwerveModulePosition getPosition(){
        return moduleIO.getPosition();
    }
    public void update(){
        moduleIO.update();
    }
}
