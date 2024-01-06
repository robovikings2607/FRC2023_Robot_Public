package frc.robot.commands.AutonCommands;

import java.util.Collections;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RVProfiledPIDController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.sim.OdomSimulation;

public class GoToPoint extends CommandBase {
    enum ThetaProfilePolicy
    {
        // Make the angular profile complete in the same time as the linear profile.
        MATCH_LINEAR,

        // Manually set constraints for the angular profile.
        MANUAL
    };

    RobotContainer robot;

    TrapezoidProfile linearProfile;

    Translation2d target;
    Rotation2d rotationTarget;

    Pose2d startPose;

    Timer profileTime = new Timer();

    double distanceToTarget;
    double trajectoryHeading;
    double angleDiff;

    double initialVelocity;
    double finalVelocity;

    static final Rotation2d ZERO_DEG = new Rotation2d(0);

    static final TrapezoidProfile.Constraints DEFAULT_LINEAR_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 1.5); //previously 1.8, 2
    static final TrapezoidProfile.Constraints DEFAULT_ANGULAR_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 360*2);
    
    TrapezoidProfile.Constraints linearProfileConstraints = DEFAULT_LINEAR_CONSTRAINTS;
    ThetaProfilePolicy thetaPolicy = ThetaProfilePolicy.MATCH_LINEAR;
    TrapezoidProfile.Constraints thetaConstraints = null;

    RVProfiledPIDController headingController = new RVProfiledPIDController(AutoConstants.kGoToPointAngularP, 0, AutoConstants.kGoToPointAngularD, DEFAULT_ANGULAR_CONSTRAINTS);

    FieldObject2d fieldTarget;

    Supplier<Pose2d> poseSupplier = null;

    public double angleTimeOffset = 0;

    public GoToPoint(RobotContainer container, Translation2d target, Rotation2d rotation, double vi, double vf)
    {
        addRequirements(container.swerveDrive);
        this.robot = container;
        this.target = target;
        this.rotationTarget = rotation;
        this.initialVelocity = vi;
        this.finalVelocity = vf;
        headingController.enableContinuousInput(-180.0, 180.0);
        fieldTarget = robot.field.getObject("GoToPointTarget");
    }

    public GoToPoint(RobotContainer container, Supplier<Pose2d> supplier)
    {
        addRequirements(container.swerveDrive);
        this.poseSupplier = supplier;
        this.robot = container;
        this.initialVelocity = 0;
        this.finalVelocity = 0;
        headingController.enableContinuousInput(-180.0, 180.0);
        fieldTarget = robot.field.getObject("GoToPointTarget");
    }

    public GoToPoint(RobotContainer container, Translation2d target, Rotation2d rotation)
    {
        this(container, target, rotation, 0, 0);
    }

    GoToPoint withLinearConstraints(TrapezoidProfile.Constraints constraints) {
        linearProfileConstraints = constraints;
        return this;
    }
    
    GoToPoint withThetaConstraints(TrapezoidProfile.Constraints constraints) {
        thetaConstraints = constraints;
        thetaPolicy = ThetaProfilePolicy.MANUAL;
        return this;
    }

    GoToPoint withAngularConstraints(TrapezoidProfile.Constraints constants) {
        return withThetaConstraints(constants);
    }

    double maxNonNanTime(double... times)
    {
        double max = 0;
        for ( var time : times )
        {
            if ( time > max && !Double.isNaN(time) )
            {
                max = time;
            }
        }
        return max;
    }


    @Override
    public void initialize() {
        robot.swerveDrive.disableRamping();
        var robotPose = robot.swerveDrive.getPose();
        if ( poseSupplier != null )
        {
            final var suppliedPose = poseSupplier.get();
            if ( suppliedPose != null )
            {
                target = suppliedPose.getTranslation();
                rotationTarget = suppliedPose.getRotation();
            }
            else
            {
                target = robotPose.getTranslation();
                rotationTarget = robotPose.getRotation();
            }

        }

        startPose = new Pose2d(robotPose.getTranslation(), ZERO_DEG);

        // Find the distance between the robot and the target.
        double linearDistance = Math.hypot(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
        trajectoryHeading = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX());

        // Create a profile over this distance.
        linearProfile = new TrapezoidProfile(linearProfileConstraints, new State(linearDistance, finalVelocity), new State(0, initialVelocity));

        // If the angular profile isn't set to manual, generate constraints that get the robot to the target angle
        // within the time that the robot takes to travel the linear distance.
        if ( thetaPolicy != ThetaProfilePolicy.MANUAL || thetaPolicy == null )
        {
            // Degrees we need to turn.
            var totalRotation = Math.abs(robotPose.getRotation().getDegrees() - rotationTarget.getDegrees());

            // There is no linear movement, default the angular constraints.
            var totalRotationDegSec = totalRotation / Math.max(linearProfile.totalTime() - angleTimeOffset, 0);
            if ( Double.isNaN(totalRotationDegSec))
            {
                totalRotationDegSec = DEFAULT_ANGULAR_CONSTRAINTS.maxVelocity;
            }

            thetaConstraints = new TrapezoidProfile.Constraints(totalRotationDegSec, DEFAULT_ANGULAR_CONSTRAINTS.maxAcceleration);
        }
        headingController.setConstraints(thetaConstraints);
        headingController.reset(new State(-robotPose.getRotation().getDegrees(), 0));
        headingController.setGoal(new State(-rotationTarget.getDegrees(), 0));
        headingController.setTolerance(AutoConstants.kMaxAngleDegreesError);

        // Start the timer to track the profile.
        profileTime.reset();
        profileTime.start();
    }

    @Override
    public void execute()
    {
        final var linearProfilePoint = linearProfile.calculate(profileTime.get());
        SmartDashboard.putNumber("linearPoint", linearProfilePoint.position);

        final var linearVelocity = new Translation2d(Math.cos(trajectoryHeading) * linearProfilePoint.velocity, Math.sin(trajectoryHeading) * linearProfilePoint.velocity);

        final var fieldTargetPose = startPose.plus(new Transform2d(new Translation2d(Math.cos(trajectoryHeading) * linearProfilePoint.position, Math.sin(trajectoryHeading) * linearProfilePoint.position), ZERO_DEG));

        final var robotPose = robot.swerveDrive.getPose();
        final var diff = fieldTargetPose.minus(robotPose);
        distanceToTarget = diff.getTranslation().getNorm();

        var feedforwardVector = linearVelocity.rotateBy(robotPose.getRotation().unaryMinus());
        
        var xPow = diff.getX() * AutoConstants.kGoToPointLinearP + feedforwardVector.getX() * AutoConstants.kGoToPointLinearF;
        var yPow = diff.getY() * AutoConstants.kGoToPointLinearP + feedforwardVector.getY() * AutoConstants.kGoToPointLinearF;
        var thetaPow = headingController.calculate(-robotPose.getRotation().getDegrees()) + headingController.getSetpoint().velocity * AutoConstants.kGoToPointAngularF;

        if ( Math.abs(yPow) > 1 )
        {
            yPow = 1.0 * Math.signum(yPow);
        }
        if ( Math.abs(xPow) > 1 )
        {
            xPow = 1.0 * Math.signum(xPow);
        }
        robot.swerveDrive.swerveDrive(-yPow, xPow, thetaPow, 1);
        
        fieldTarget.setPose(new Pose2d(fieldTargetPose.getTranslation(), Rotation2d.fromDegrees(-headingController.getSetpoint().position)).relativeTo(Constants.LEFT_BOTTOM_CORNER));

        if (Robot.isSimulation())
        {
            OdomSimulation.getInstance().update(new Pose2d(fieldTargetPose.getTranslation(), Rotation2d.fromDegrees(-headingController.getSetpoint().position)));
        }
    }

    @Override
    public boolean isFinished()
    {
        // Look at whichever profile takes longer to execute.
        var totalTrajectoryLength = maxNonNanTime(linearProfile.totalTime(), headingController.totalTime());

        // If the profile hasn't finished yet, keep running.
        if( profileTime.get() < totalTrajectoryLength)
        {
            return false;
        }

        // If the profile has ended and we're still sitting after a timeout, abort.
        var timeoutTime = totalTrajectoryLength;
        if ( finalVelocity == 0 )
        {
            timeoutTime += AutoConstants.maxTrajectoryOverrunSeconds;
        }
        if (profileTime.get() > timeoutTime )
        {
            return true;
        }

        // Exit if we're at the target.
        return Math.abs(distanceToTarget) < AutoConstants.kMaxDistanceMetersError && headingController.atGoal(); // && Math.abs(angleDiff) < AutoConstants.kMaxAngleDegreesError;
    }

    @Override
    public void end(boolean interrupted)
    {
        if ( finalVelocity == 0 )
        {
            robot.swerveDrive.swerveDrive(0, 0, 0, 1);
        }
        fieldTarget.setPoses(Collections.emptyList());
    }

    public GoToPoint setAngleTimeOffset(double time) 
    {
        angleTimeOffset = time;
        return this;
    }
}
