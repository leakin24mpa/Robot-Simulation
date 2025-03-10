package frc.robot.subsystems.wrist.wristIO;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.WristConstants;


public class NeoWristIO implements WristIO{
    private SparkMax motor = new SparkMax(WristConstants.motorID, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();


    public NeoWristIO(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        
        config.encoder.positionConversionFactor(WristConstants.gearRatio);
        config.encoder.velocityConversionFactor(WristConstants.gearRatio / 60);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
    }
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromDegrees(encoder.getVelocity());
    }
    public void update(){

    }
    
}
