package frc.robot.subsystems.swerve.odometry;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.SwerveConstants;

public class Pigeon2Odometry extends SwerveOdometryIO{

    private Pigeon2 gyro;
    public Pigeon2Odometry(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> posititionsSupplier, int pigeonID){
        super(kinematics, posititionsSupplier);
        gyro = new Pigeon2(pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        
        
    }
    @Override
    public Rotation2d getGyroHeading(){
        if(SwerveConstants.invertGyro){
            return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
        }
        else{
            return Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble());
        }
        
    }
    @Override
    public void setGyroHeading(Rotation2d heading){
        gyro.setYaw(heading.getDegrees());
    }

}
