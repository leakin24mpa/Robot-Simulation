package frc.robot.subsystems.swerve.odometry;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class SimOdometry extends SwerveOdometryIO{

    private SwerveModulePosition[] lastPositions;
    private SwerveDriveKinematics kinematics;
    private Rotation2d simulatedHeading;

    public SimOdometry(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> posititionsSupplier){
        super(kinematics, posititionsSupplier);
        this.kinematics = kinematics;
        this.lastPositions = posititionsSupplier.get();
        this.simulatedHeading = new Rotation2d();
    }
    @Override
    public Rotation2d getGyroHeading(){
        return simulatedHeading;
    }
    @Override
    public void setGyroHeading(Rotation2d heading){
        simulatedHeading = heading;
    }

    
    @Override
    public void update() {
        SwerveModulePosition[] newPositions = super.positionsSupplier.get();
        Rotation2d twist = new Rotation2d(kinematics.toTwist2d(lastPositions, newPositions).dtheta);
  
        simulatedHeading = simulatedHeading.plus(twist);

        lastPositions = newPositions;
        super.update(simulatedHeading, newPositions);
    }

}
