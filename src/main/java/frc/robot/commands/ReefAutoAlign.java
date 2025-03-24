package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.FieldConstants.*;
import frc.robot.subsystems.swerve.SwerveDrive;


public class ReefAutoAlign extends Command{
    private SwerveDrive m_Drive;
    private PIDController rotationPID = new PIDController(0.3, 0, 0);
    private PIDController xPID = new PIDController(5, 0, 0.01);
    private PIDController yPID = new PIDController(5, 0, 0.01);

    private boolean isRightSide;

    private final double maxOutput = 2;
    private final StructPublisher<Pose2d> goalPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("AutoAlign Goal Pose", Pose2d.struct).publish();


    public ReefAutoAlign(SwerveDrive drive, boolean isRightSide){
        m_Drive = drive;
        addRequirements(drive);

        this.isRightSide = isRightSide;
        rotationPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute(){
        ChassisSpeeds outputSpeeds = calculate(m_Drive.getRobotPose(), flipForAlliance(BLUE_REEF_CENTER), 1.3, 0.15, isRightSide);
        m_Drive.driveFromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(outputSpeeds, m_Drive.getYaw()), true);
    }

    private ChassisSpeeds calculate(Pose2d currentPose, Translation2d target, double scoreDistance, double scoreOffset, boolean isRightSide){
        Translation2d offset = currentPose.getTranslation().minus(target);
        double goalAngle = Math.round((offset.getAngle().getDegrees()) / 60) * 60; 
        Rotation2d goalRotation = Rotation2d.fromDegrees(goalAngle);

        double nx = goalRotation.getCos();
        double ny = goalRotation.getSin();

        double tx = -ny;
        double ty = nx;

        if(isRightSide){
            scoreOffset *= -1;
        }

        double goalX = target.getX() + scoreDistance * nx + scoreOffset * tx;
        double goalY = target.getY() + scoreDistance * ny + scoreOffset * ty;

        double rotationOutput = rotationPID.calculate(currentPose.getRotation().getDegrees(), goalAngle + 180);
        
        double xOutput = xPID.calculate(currentPose.getX(), goalX);
        double yOutput = yPID.calculate(currentPose.getY(), goalY);

        xOutput = MathUtil.clamp(xOutput, -maxOutput, maxOutput);
        yOutput = MathUtil.clamp(yOutput, -maxOutput, maxOutput);
        

        goalPosePublisher.set(new Pose2d(goalX, goalY, goalRotation));
        return new ChassisSpeeds(xOutput, yOutput, rotationOutput);
    }
}
