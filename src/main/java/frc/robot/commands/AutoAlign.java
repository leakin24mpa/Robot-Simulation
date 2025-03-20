package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlign {
    private PIDController rotationPID = new PIDController(0, 0, 0);
    private PIDController xPID = new PIDController(1, 0, 0);
    //private PIDController yPID = new PIDController(0, 0, 0);


    public AutoAlign(){
        rotationPID.enableContinuousInput(-180, 180);
    }
    public class AutoAlignOutput{
        public double xOutput = 0;
        public double yOutput = 0;
        public double rotationOutput = 0;
    }
    public AutoAlignOutput calculate(Pose2d currentPose, Translation2d target, double goalDistance, double strafeInput){
        Translation2d offset = currentPose.getTranslation().minus(target);
        double goalAngle = Math.round((offset.getAngle().getDegrees()) / 60) * 60; 
        Rotation2d goalRotation = Rotation2d.fromDegrees(goalAngle);

        double nx = goalRotation.getCos();
        double ny = goalRotation.getSin();

        double tx = -ny;
        double ty = nx;

        double distanceToGoal = offset.getX() * nx + offset.getY() * ny;
        double strafeToGoal = offset.getX() * tx + offset.getY() * ty;



        double distanceControl = xPID.calculate(distanceToGoal, goalDistance);
        double strafeControl = strafeInput * 0.4;
        double rotationControl = rotationPID.calculate(currentPose.getRotation().getDegrees(), goalAngle + 180);
        
        AutoAlignOutput o = new AutoAlignOutput();
        o.xOutput = strafeControl * tx;
        o.yOutput = strafeControl * ty;
        if(Math.abs(strafeToGoal) > goalDistance / 2){
            o.xOutput += offset.getX() / offset.getNorm() * distanceControl;
            o.yOutput += offset.getY() / offset.getNorm() * distanceControl;
        }
        else{
            o.xOutput += distanceControl * nx;
            o.yOutput += distanceControl * ny;
        }
        o.rotationOutput = rotationControl;

        SmartDashboard.putNumber("AutoAlign/distanceToGoal", distanceToGoal);
        SmartDashboard.putNumber("AutoAlign/Xoutput", o.xOutput);
        SmartDashboard.putNumber("AutoAlign/Youtput", o.yOutput);
        SmartDashboard.putNumber("AutoAlign/nx", nx);
        SmartDashboard.putNumber("AutoAlign/ny", ny);
        SmartDashboard.putNumber("AutoAlign/goalAngle", goalAngle);
        return o;
    }
}
