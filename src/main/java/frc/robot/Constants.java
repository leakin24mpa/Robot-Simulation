package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public class SwerveConstants {
        public static final int gyro_ID = 17;
        public static final boolean invertGyro = true;

        /* Drivetrain Constants */
        public static final double halfTrackWidth = Units.inchesToMeters(21.0 / 2.0); //half of the left-right distance between the wheels
        public static final double halfWheelBase = Units.inchesToMeters(21.0 / 2.0 ) ; //half of the forward-backward distance between the wheels
        public static final double driveBaseRadius =  Math.hypot(halfTrackWidth,halfWheelBase); //distance from the robot center to each wheel

        public static final double wheelDiameter = (.0992); // (Meters) This can change as the wheels wear down. It's a good idea to re-measure this regularly to keep odometry accurate
        public static final double wheelCircumference = wheelDiameter * Math.PI; // Meters

        
        public static final double driveGearRatio = (8.14 / 1.0); // 8.14:1 ( SDS Mk4 L1 Module )
        //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 ( SDS Mk4 L1 Module ) 
        //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1



        /* Drive Motor Conversion Factors */
        //conversion from encoder rotations to meters driven
        public static final double driveConversionPositionFactor = (wheelCircumference) / driveGearRatio;
        //conversion from encoder RPMs to meters per second
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        //conversion from encoder rotations to wheel angle degrees
        public static final double angleConversionFactor = 360.0 / angleGearRatio;
    
        /* Maximum speed and angular velocity of the robot */
        public static final double maxSpeed = 5; // meters per second (measure with real robot or estimate with reCalc)
        public static final double maxAngularVelocity = maxSpeed / driveBaseRadius; //radians per second

        /* Slew rate limits for joystick inputs. lower numbers = smoother driving, higher numbers = more responsive*/
        public static final double translationSlewRateLimit = 3;
        public static final double rotationSlewRateLimit = 3;
        

        /* Drive Motor Voltage Compensation */
        //this keeps the motor speeds consistent even as the battery voltage changes.
        public static final double voltageComp = 12.0;
        
        //Motor Current Limits
        public static final int angleContinuousCurrentLimit = 20; //limits current draw of angle motor
        public static final int driveContinuousCurrentLimit = 50; //limits current draw of drive motor
    
        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; 
        public static final double driveKI = 0.0; 
        public static final double driveKD = 0.0; 

        /* Drive Motor Feedforward Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44; 
        public static final double driveKA = 0.5; 

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01; 
        public static final double angleKI = 0.0; 
        public static final double angleKD = 0.0; 

        //invert the direction of the motors using these booleans 
        public static final boolean angleMotorInvert = false;
        public static final boolean driveMotorInvert = false;

        public enum ModuleLocation{
            FRONT_LEFT(new Translation2d(halfTrackWidth, halfWheelBase)),
            BACK_LEFT(new Translation2d(-halfTrackWidth, halfWheelBase)),
            BACK_RIGHT(new Translation2d(-halfTrackWidth, -halfWheelBase)),
            FRONT_RIGHT(new Translation2d(halfTrackWidth, -halfWheelBase));

            public final Translation2d position;
            ModuleLocation(Translation2d position) {
                this.position = position;
            }
        }
        public record ModuleData(
            int driveMotorID, int angleMotorID, int encoderID, double angleOffset, ModuleLocation location
        ){
            public Translation2d getLocation(){
                return location.position;
            }
        }
        
        public static ModuleData[] moduleData = {
            //Module 0: Front Left
            new ModuleData(3, 2, 11, 160.2, ModuleLocation.FRONT_LEFT),

            //Module 1: Back Left
            new ModuleData(5, 4, 12, 117.2, ModuleLocation.BACK_LEFT),

            //Module 2: Back Right
            new ModuleData(7, 6, 13, 141, ModuleLocation.BACK_RIGHT),

            //Module 3: Front RIght
            new ModuleData(9, 8, 14, -138, ModuleLocation.FRONT_RIGHT),
        };

       //give location of each module relative to robot center in meters to a swerveDriveKinematics
        //this is the x,y coordinate of each wheel. 
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            moduleData[0].getLocation(), 
            moduleData[1].getLocation(),
            moduleData[2].getLocation(),
            moduleData[3].getLocation()
        );
       
    }
    
    public class AutoConstants {
        public static final ModuleConfig ppModuleConfig = 
            new ModuleConfig(SwerveConstants.wheelDiameter / 2,
                            SwerveConstants.maxSpeed,
                            1.2,
                            DCMotor.getNeoVortex(1).withReduction(SwerveConstants.driveGearRatio),
                            SwerveConstants.driveContinuousCurrentLimit,
                            1);
        
        public static final RobotConfig ppConfig = new RobotConfig(75,6.8,ppModuleConfig, 
            SwerveConstants.moduleData[0].getLocation(),
            SwerveConstants.moduleData[1].getLocation(),
            SwerveConstants.moduleData[2].getLocation(),
            SwerveConstants.moduleData[3].getLocation());

        public static final PPHolonomicDriveController ppSwerveController = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.00001, 0.0), // Translation PID constants for path following
            new PIDConstants(5.0, 0.0005, 0.001));

    }
    public class ElevatorConstants{
        public static final int elevatorMotorID = 0;
        public static final int upperLimitID = 0;
        public static final int lowerLimitID = 1;

        public static final double minPosition = 0;
        public static final double maxPosition = 2;

        public static final double gearRatio = 3.0;
        public static final double spoolRadius = 0.01;
        public static final double spoolCircumference = 2 * Math.PI * spoolRadius;
        public static final double positionConversionFactor = spoolCircumference * gearRatio / 360;
        public static final double velocityConversionFactor = spoolCircumference * gearRatio / 60;

        public static double kP = 5;
        public static double kI = 0;
        public static double kD = 0;

        public static PIDController PID = new PIDController(kP, kI, kD);

        public static double kS = 0;
        public static double kG = 0.0753;
        public static double kV = 0.53;

        public static ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);

        public static double maxVelocity = 3.5;
        public static double maxAcceleration = 0.8;

        public static Constraints constraints = new Constraints(maxVelocity, maxAcceleration);

    }
    public class WristConstants{
        public static final int motorID = 0;

        public static final double gearRatio = 100;
        
        public static final double kP = 0.005;
        public static final double kI = 0.001;
        public static final double kD = 0;
        public static final PIDController PID = new PIDController(kP, kI, kD);


    }

    public class FieldConstants {
        public static final double FIELD_LENGTH = 17.54824934;
        public static final double FIELD_WIDTH = 8.052;

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.48933684,4.02587697);

        public static final Rotation2d RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(234.011392);
        public static final Rotation2d LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-234.011392);

        public static boolean isRedAlliance(){
            return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        }
        
        public static Rotation2d flipForAlliance(Rotation2d rotation){
            if(isRedAlliance()){
                return Rotation2d.fromDegrees(rotation.getDegrees() + 180);
            }else{
                return rotation;
            }
        }
        public static Translation2d flipForAlliance(Translation2d pos){
            if(isRedAlliance()){
                return new Translation2d(FIELD_LENGTH - pos.getX(), FIELD_WIDTH - pos.getY());
            }else{
                return pos;
            }
        }
        public static Pose2d flipForAlliance(Pose2d pose){
            return new Pose2d(flipForAlliance(pose.getTranslation()), flipForAlliance(pose.getRotation()));
        }
    }
}
