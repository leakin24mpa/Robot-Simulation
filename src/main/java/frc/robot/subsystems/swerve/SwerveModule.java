// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CANSparkUtil;
import frc.lib.CANSparkUtil.Usage;

/** Add your docs here. */
public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final CANcoder absoluteEncoder;
    private final Rotation2d angleOffset;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;

    private final SimpleMotorFeedforward feedforward;

    public final int moduleNumber;
    
    public SwerveModule(int moduleNumber, int driveID, int angleID, int encoderID, double angleOffsetDegrees){
        //identifies which of the four modules this is
        this.moduleNumber = moduleNumber;
        //used for calibrating the absolute encoder so it knows which direction is zero
        this.angleOffset = Rotation2d.fromDegrees(angleOffsetDegrees);

        //one motor drives the wheel, the other motor steers it 
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleID,MotorType.kBrushless);

        //encoders measure how far the wheel has rolled and in what direction it's facing
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        //the absolute encoder tells the relative encoder which way the wheel is facing when the code first starts up
        absoluteEncoder = new CANcoder(encoderID);

        //PID & feedforward  controllers bring the wheel to its desired direction and speed
        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();
        feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

        //configure the motors with correct settings and reset the encoders.
        configureMotors();
    }
    
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        //find how far off the wheel's actual direction is from its target direction
        double difference = desiredState.angle.minus(currentAngle).getDegrees();
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
        //return the updated speed and direction
        return new SwerveModuleState(speed, currentAngle.plus(Rotation2d.fromDegrees(turnAmount)));
    }
    /** Set the goal state of the module to a speed and direction */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //optimize the desired state to get the module to its goal as fast as possible
        SwerveModuleState optimizedState = optimize(desiredState, getAngle());
        
        //Skip updating the angle of the modules if the speed is really slow
        if(optimizedState.speedMetersPerSecond > 0.01 * SwerveConstants.maxSpeed){
            setAngle(optimizedState.angle);
        }
        
        setSpeed(optimizedState.speedMetersPerSecond, isOpenLoop);
    }
    //set the goal direction of the module
    private void setAngle(Rotation2d angle){
        anglePID.setReference(angle.getDegrees(), ControlType.kPosition);
    }
    //set the speed of the module
    private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop){
        if(isOpenLoop){
            //estimate the motor voltage needed for the desired speed (don't correct with PID, since this is open loop control)
            double percentOutput = speedMetersPerSecond / SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else{
            //use the feedforward controller to estimate the motor voltage needed for the desired speed
            double feedforwardOutput = feedforward.calculate(speedMetersPerSecond);
            //combine the feedforward output with the PID output
            drivePID.setReference(speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardOutput);
        }
    }
    


    //get the angle from the absolute encoder
    public Rotation2d getAbsoluteEncoderAngle(){
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }
    //get the angle from the main encoder
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    private void configureMotors(){
        
        
        //reset all settings to default to clear out any old data  
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig angleConfig = new SparkMaxConfig(); 


        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

        //minimize the update frequency of the CANcoder to avoid clogging the CAN bus with unnecessary data (we only need this data once at the very beginning)
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(4);
        //send all drive motor data over CAN bus (we need both position and velocity of the drive motor)
        CANSparkUtil.setCANSparkBusUsage(driveConfig, Usage.kAll);
        //limit turn motor CAN bus usage to only position (we don't care about the angle motor's velocity, just its position)
        CANSparkUtil.setCANSparkBusUsage(angleConfig, Usage.kPositionOnly);

        //set current limits
        driveConfig.smartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        angleConfig.smartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);

        //voltage compensation corrects for differences in speed due to varying battery voltage
        driveConfig.voltageCompensation(SwerveConstants.voltageComp);
        angleConfig.voltageCompensation(SwerveConstants.voltageComp);

        //set motor forward directions
        driveConfig.inverted(SwerveConstants.driveMotorInvert);
        angleConfig.inverted(SwerveConstants.angleMotorInvert);

        //keep swerve motors in brake mode so the robot actually stops when you want it to
        driveConfig.idleMode(IdleMode.kBrake);
        angleConfig.idleMode(IdleMode.kBrake);

        //set drive encoder to return velocity as meters per second
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        //set drive  encoder to return position as meters
        driveConfig.encoder.positionConversionFactor(SwerveConstants.driveConversionPositionFactor);

        //set angle encoder so it measures wheel rotations instead of motor rotations
        angleConfig.encoder.positionConversionFactor(SwerveConstants.angleConversionFactor);

        //send PID values to the motor controllers
        ClosedLoopConfig drivePIDconfig = new ClosedLoopConfig();
        ClosedLoopConfig anglePIDconfig = new ClosedLoopConfig();
        drivePIDconfig.pid(SwerveConstants.driveKP,SwerveConstants.driveKI, SwerveConstants.driveKD);
        anglePIDconfig.pid(SwerveConstants.angleKP,SwerveConstants.angleKI, SwerveConstants.angleKD);

        driveConfig.apply(drivePIDconfig);
        angleConfig.apply(anglePIDconfig);

        

        //save all data to the motor controllers
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //resets drive encoder distance to 0m
        driveEncoder.setPosition(0.0);

        //wait a second to give time for the absolute encoder to start up
        Timer.delay(1.0);
        
        //reset angle encoder position to match the absolute encoder
        Rotation2d absoluteAngle = getAbsoluteEncoderAngle().minus(angleOffset);
        angleEncoder.setPosition(absoluteAngle.getDegrees());
    }
    
    

}
