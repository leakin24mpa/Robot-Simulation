package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveConstants;

public class SimModuleIO implements SwerveModuleIO{
    private final DCMotor driveMotor = DCMotor.getNeoVortex(1);
    private final DCMotor angleMotor = DCMotor.getNEO(1);

    
    private void motorSimSet(DCMotorSim sim, double speed){
        sim.setInputVoltage(Math.min(Math.max(12.0 * speed, -12.0), 12.0));
    }
    private final DCMotorSim driveWheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveMotor, 0.025,SwerveConstants.driveGearRatio),
        driveMotor);
    private final DCMotorSim angleWheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(angleMotor, 0.004,SwerveConstants.angleGearRatio),
        driveMotor);
    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV);
    private final PIDController drivePID = new PIDController(SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD);
    private final PIDController anglePID = new PIDController(SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD);

    public SimModuleIO(){

    }


    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(angleWheelSim.getAngularPositionRad());
    }
    private double getVelocity(){
        return driveWheelSim.getAngularVelocityRPM() * SwerveConstants.wheelCircumference / 60;
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveWheelSim.getAngularPositionRotations() * SwerveConstants.wheelCircumference, getAngle());
    }

    public void setAngle(Rotation2d angle){
        motorSimSet(angleWheelSim, anglePID.calculate(getAngle().getDegrees(), angle.getDegrees()));
    }
    public void setSpeed(double speedMetersPerSecond, boolean isOpenLoop){
        if(isOpenLoop){
            motorSimSet(driveWheelSim, speedMetersPerSecond / SwerveConstants.maxSpeed);
        }
        else{
            motorSimSet(driveWheelSim, feedforward.calculate(speedMetersPerSecond) + drivePID.calculate(getVelocity(), speedMetersPerSecond));
        }
    }
    public void update(){
        driveWheelSim.update(0.02);
        angleWheelSim.update(0.02);
    }
}
