package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    public void setSpeed(double speedMetersPerSecond, boolean isOpenLoop);
    public void setAngle(Rotation2d angle);

    public void update();

    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public Rotation2d getAngle();
}
