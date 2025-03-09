package frc.robot.subsystems.swerve.moduleIO;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CANSparkUtil;
import frc.lib.CANSparkUtil.Usage;
import frc.robot.SwerveConstants;

public class NeoModuleIO implements SwerveModuleIO{
    private final SparkFlex driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final CANcoder absoluteEncoder;
    private final Rotation2d angleOffset;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;

    private final SimpleMotorFeedforward feedforward;

    public NeoModuleIO(int driveID, int angleID, int encoderID, double angleOffsetDegrees){
        //used for calibrating the absolute encoder so it knows which direction is zero
        this.angleOffset = Rotation2d.fromDegrees(angleOffsetDegrees);

        //one motor drives the wheel, the other motor steers it 
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
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

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
    //get the angle from the absolute encoder
    private Rotation2d getAbsoluteEncoderAngle(){
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }

    public void setSpeed(double speedMetersPerSecond, boolean isOpenLoop){
        if(isOpenLoop){
            driveMotor.set(speedMetersPerSecond/SwerveConstants.maxSpeed);
        }else{
            drivePID.setReference(speedMetersPerSecond, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(speedMetersPerSecond));
        }
    }
    public void setAngle(Rotation2d angle){
        anglePID.setReference(angle.getDegrees(), ControlType.kPosition);
    }
    public void update(){

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
