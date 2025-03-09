package frc.robot.subsystems.elevator.elevatorIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class NeoElevatorIO implements ElevatorIO{
    private SparkMax motor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
    private RelativeEncoder encoder;

    private DigitalInput lowerBound = new DigitalInput(ElevatorConstants.lowerLimitID);
    private DigitalInput upperBound = new DigitalInput(ElevatorConstants.upperLimitID);

    public NeoElevatorIO(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        
        config.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);
        config.encoder.velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
    }
   
    public double getPosition() {
        return encoder.getPosition();
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }
    public TrapezoidProfile.State getState(){
        return new TrapezoidProfile.State(getPosition(), getVelocity());
    }
    public boolean isAtUpperBound(){
        return upperBound.get();
    }
    public boolean isAtLowerBound(){
        return lowerBound.get();
    }
    public void resetPosition(double newPosition) {
        encoder.setPosition(newPosition);
    }
    
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    
    public void update() {
        
    }

    
}
