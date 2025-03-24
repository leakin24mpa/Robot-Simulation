package frc.robot.subsystems.swerve;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class Odometry extends SwerveDrivePoseEstimator{
    public boolean isSimGyro;

    private SwerveDriveKinematics kinematics;
    private Rotation2d simulatedHeading;
    private SwerveModulePosition[] lastPositions;

    public Odometry(SwerveDriveKinematics kinematics, SwerveModulePosition[] startingPostions, Pose2d startingPose, boolean useSimGyro){
        super(kinematics, startingPose.getRotation(), startingPostions, startingPose);
        simulatedHeading = startingPose.getRotation();
        lastPositions = startingPostions;
        this.kinematics = kinematics;
        isSimGyro = useSimGyro;
    }
    @Override
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        Rotation2d heading = gyroAngle;
        Rotation2d twist = new Rotation2d(kinematics.toTwist2d(lastPositions, wheelPositions).dtheta);
        if(isSimGyro){
            simulatedHeading = simulatedHeading.plus(twist);
            heading = simulatedHeading;
        }
        lastPositions = wheelPositions;
        return super.update(heading, wheelPositions);
    }
}
