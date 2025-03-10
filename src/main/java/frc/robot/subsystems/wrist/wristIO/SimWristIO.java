package frc.robot.subsystems.wrist.wristIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class SimWristIO implements WristIO{
    private DCMotor wristMotor = DCMotor.getNEO(1);
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        wristMotor, 
        WristConstants.gearRatio, 
        0.04, 
        0.25, 
        -Math.PI/2, Math.PI/2, 
        true, 
        0);
    public SimWristIO(){

    }
    public void setSpeed(double speed){
        armSim.setInputVoltage(Math.min(Math.max(12.0 * speed, -12.0), 12.0));
    }
    public Rotation2d getAngularVelocity(){
        return Rotation2d.fromRadians(armSim.getVelocityRadPerSec());
    }
    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(armSim.getAngleRads());
    }
    public void update(){
        armSim.update(0.02);
    }
}
