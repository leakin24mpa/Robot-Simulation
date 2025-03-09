package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class AutoConstants {
    public static final ModuleConfig ppModuleConfig = 
        new ModuleConfig(SwerveConstants.wheelDiameter / 2,
                         SwerveConstants.maxSpeed,
                         1.2,
                         DCMotor.getNeoVortex(1).withReduction(SwerveConstants.driveGearRatio),
                         SwerveConstants.driveContinuousCurrentLimit,
                         1);
    static double length = SwerveConstants.halfWheelBase;
    static double width = SwerveConstants.halfTrackWidth;
    public static final RobotConfig ppConfig = new RobotConfig(75,6.8,ppModuleConfig, 
        new Translation2d(length, width), 
        new Translation2d(-length, width),
        new Translation2d(-length, -width),
        new Translation2d(length, -width));

    public static final PPHolonomicDriveController ppSwerveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.00001, 0.0), // Translation PID constants for path following
        new PIDConstants(5.0, 0.0005, 0.001));

}
