package frc.robot.commands.AutonCommands.AutonUtilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveOnChargeStation extends CommandBase {
    RobotContainer robot;
    double driveSpeed;
    boolean never_on = true;
    Timer overdrive = new Timer();
    public DriveOnChargeStation(RobotContainer robot, double driveSpeed)
    {
        addRequirements(robot.swerveDrive);
        this.robot = robot;
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {
        overdrive.reset();
        never_on = true;
    }

    @Override
    public void execute()
    {
        robot.swerveDrive.swerveDrive(0, this.driveSpeed, 0, 1);
        if ( onChargeStation() && never_on )
        {
            overdrive.reset();
            overdrive.start();
            never_on = false;
        }
    }

    @Override
    public boolean isFinished()
    {
        return overdrive.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted)
    {
        robot.swerveDrive.swerveDrive(0, 0, 0, 0);
    }

    boolean onChargeStation()
    {
        double hypot = Math.hypot(robot.swerveDrive.getPitch(), robot.swerveDrive.getRoll());
        return hypot > DriveConstants.ON_BALANCE_TOLERANCE;
    }
}
