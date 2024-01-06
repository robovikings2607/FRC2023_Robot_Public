package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.subsystems.ArmSubsystem;

public class SequentialArmUp extends CommandBase{
    private RobotContainer robot;
    private ArmSubsystem arm;
    private boolean yield = false;
    private ArmPresets armPreset;
    private double shoulderDestination, elbowDestination;

    public SequentialArmUp(RobotContainer robotContainer, ArmPresets armPreset) {
        robot = robotContainer;
        arm = robot.arm;
        addRequirements(arm);

        this.armPreset = armPreset;
    }

    @Override
    public void initialize() {
        if (arm.getCurrentPreset() != ArmPresets.STOWING)
        {
            cancel();
        }

        switch(armPreset) {
            case SCORING_HIGH :
                shoulderDestination = ArmConstants.SHOULDER_HIGH_CONE;
                elbowDestination = ArmConstants.ELBOW_HIGH_CONE;
                break;
            case SCORING_MID :
                shoulderDestination = ArmConstants.COOL_SHOULDER_MID_CONE;
                elbowDestination = ArmConstants.COOL_ELBOW_MID_CONE;
        }

        arm.shoulderPIDController.reset(arm.shoulderEncoder.getAbsolutePosition());
        arm.elbowPIDController.reset(arm.elbowEncoder.getAbsolutePosition());

        arm.shoulderPIDController.setGoal(shoulderDestination);
        arm.elbowPIDController.setGoal(ArmConstants.ELBOW_STOWING);
    }

    @Override
    public void execute() {
        //Angle stuff
        if (arm.getTrueForearmAngle().getCos() < 0.0 || arm.shoulderPIDController.atGoal())
        {
            arm.elbowPIDController.setGoal(elbowDestination);
        }
    }

    @Override
    public boolean isFinished() {
        return 
            approxEqual(arm.shoulderPIDController.getGoal().position, shoulderDestination) && 
            approxEqual(arm.elbowPIDController.getGoal().position, elbowDestination) &&
            (!yield || (arm.elbowPIDController.atGoal() && arm.shoulderPIDController.atGoal()));
    }

    boolean approxEqual(double a, double b)
    {
        return Math.abs(a-b) < 0.00001;
    }



    @Override
    public void end(boolean i) {
        arm.setCurrentPreset(ArmPresets.SCORING_HIGH);
    }

    public SequentialArmUp yield() {
        this.yield = true;
        return this;
    }
}