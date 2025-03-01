package frc.robot.subsystems.swerve;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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


    //give location of each module relative to robot center in meters to a swerveDriveKinematics
    //this is the x,y coordinate of each wheel. 
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(halfWheelBase, halfTrackWidth), 
      new Translation2d(-halfWheelBase, halfTrackWidth),
      new Translation2d(-halfWheelBase, -halfTrackWidth),
      new Translation2d(halfWheelBase, -halfTrackWidth)
    );
    

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

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3; 
      public static final int angleMotorID = 2; 
      public static final int encoderID = 11;
      public static final double angleOffset = 160.2;
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int encoderID = 12;
      public static final double angleOffset = 117.2;
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 6;
      public static final int encoderID = 13;
      public static final double angleOffset = 141;
    }

    /* Front Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 8;
      public static final int encoderID = 14;
      public static final double angleOffset = -138;
    }
}

