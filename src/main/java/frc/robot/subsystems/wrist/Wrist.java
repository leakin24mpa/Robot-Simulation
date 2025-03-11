package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.wrist.wristIO.NeoWristIO;
import frc.robot.subsystems.wrist.wristIO.SimWristIO;
import frc.robot.subsystems.wrist.wristIO.WristIO;

public class Wrist extends SubsystemBase{
    private final WristIO wristIO;
    private final PIDController pid = WristConstants.PID;
    private final ArmFeedforward feedforward = WristConstants.feedforward;
    private double setpoint;
    
    public Wrist(){
        if(RobotBase.isSimulation()){
            wristIO = new SimWristIO();
        }
        else{
            wristIO = new NeoWristIO();
        }

        setpoint = wristIO.getAngle().getDegrees();
    }
    public Rotation2d getAngle(){
        return wristIO.getAngle();
    }
    public void setSetpoint(double angle){
        setpoint = angle;
    }
    public Command setSetpointCommand(double angle){
        return runOnce(() -> {
            setpoint = angle;
        });
    }
    public boolean isSafe(){
        return getAngle().getDegrees() <= ScoringConstants.safeZoneMaxAngle;
    }

    public boolean isAtSetpoint(){
        return Math.abs(setpoint - wristIO.getAngle().getDegrees()) < 1;
    }

    @Override
    public void periodic(){
        double output = feedforward.calculate(setpoint, 0) + pid.calculate(wristIO.getAngle().getDegrees(), setpoint);
        wristIO.setSpeed(output);
        wristIO.update();


        SmartDashboard.putNumber("Wrist Setpoint", Rotation2d.fromDegrees(setpoint).getRadians());


        SmartDashboard.putNumber("Wrist Actual Velocity", wristIO.getAngularVelocity().getRadians());
        SmartDashboard.putNumber("Wrist Actual Position", wristIO.getAngle().getRadians());

        SmartDashboard.putBoolean("Wrist Is At Setpoint", isAtSetpoint());

        SmartDashboard.putNumber("Wrist Motor Output", output);
    }


}
