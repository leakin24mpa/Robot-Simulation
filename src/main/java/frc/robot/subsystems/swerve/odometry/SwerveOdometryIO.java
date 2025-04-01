package frc.robot.subsystems.swerve.odometry;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveOdometryIO extends SwerveDrivePoseEstimator{
    public class VisionData{
        Pose2d pose;
        double timestampSeconds;
        double standardDeviation;
    }
    
    protected Supplier<SwerveModulePosition[]> positionsSupplier;
    protected SwerveOdometryIO(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> posititionsSupplier){
        super(kinematics, new Rotation2d(), posititionsSupplier.get(), new Pose2d());
        this.positionsSupplier = posititionsSupplier;
        

    }
    public Rotation2d getGyroHeading(){
        return new Rotation2d();
    }
    public void setGyroHeading(Rotation2d heading){}

    public Pose2d getRobotPose(){
        return super.getEstimatedPosition();
    }
    public void setRobotPose(Pose2d pose){
        super.resetPosition(pose.getRotation(), positionsSupplier.get(), pose);
        setGyroHeading(pose.getRotation());
    }

    public void updateWithVisionData(VisionData data){
        super.setVisionMeasurementStdDevs(VecBuilder.fill(data.standardDeviation, data.standardDeviation, 9999999));
        super.addVisionMeasurement(data.pose, data.timestampSeconds);
    }
    public void update(){
        super.update(getGyroHeading(), positionsSupplier.get());
    }
    

}
