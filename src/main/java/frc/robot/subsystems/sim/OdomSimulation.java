package frc.robot.subsystems.sim;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Robot;

public class OdomSimulation {
    final static OdomSimulation instance = new OdomSimulation();

    SwerveDrivePoseEstimator odom;

    final SwerveModulePosition[] zero_module_positions = new SwerveModulePosition[]{
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    public static OdomSimulation getInstance()
    {
        if (Robot.isReal())
        {
            return null;
        }
        return instance;
    }

    // Prevent public construction
    private OdomSimulation() {}

    public void attachOdom(SwerveDrivePoseEstimator odom)
    {
        this.odom = odom;
    }

    public void update(Pose2d pose)
    {
        if (odom != null)
        {
            odom.resetPosition(new Rotation2d(), zero_module_positions, pose);
        }
    }
}
