package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LED.LEDColor;

public class SetArmPreset extends CommandBase{
    private RobotContainer robot;
    private ArmPresets set;
    private double shoulderGoal, elbowGoal, shoulderStartPos;
    private boolean gamePieceProtection;
    private ArmSubsystem arm;
    private boolean coneMode, yield;

    public SetArmPreset(RobotContainer robotContainer, ArmPresets preset) {
        robot = robotContainer;
        set = preset;
        arm = robot.arm;
        yield = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.shoulderPIDController.reset(arm.shoulderEncoder.getAbsolutePosition());
        arm.elbowPIDController.reset(arm.elbowEncoder.getAbsolutePosition());

        gamePieceProtection = (arm.getCurrentPreset() == ArmPresets.INTAKING && set == ArmPresets.STOWING);

        coneMode = robot.intake.getConeMode();
        SmartDashboard.putBoolean("SettingArm", true);
        switch(set) {
            case STOWING :
                shoulderGoal = ArmConstants.SHOULDER_STOWING;
                elbowGoal = ArmConstants.ELBOW_STOWING;
                break;
            case HYBRID_INTAKING:
                shoulderGoal = ArmConstants.SHOULDER_HYBRID_INTAKING;
                elbowGoal = ArmConstants.ELBOW_HYBRID_INTAKING;
                break;
            case INTAKING :
                shoulderGoal = ArmConstants.SHOULDER_INTAKE;
                elbowGoal = ArmConstants.ELBOW_INTAKE;
                break;
            case HP_INTAKE :
                robot.intake.setConeMode(true);
                robot.intake.setSolenoid(IntakeConstants.CONE_GRAB_STATE);
                robot.led.setLED(LEDColor.YELLOW);
                shoulderGoal = ArmConstants.SHOULDER_HP;
                elbowGoal = ArmConstants.ELBOW_HP;
                break;
            case SCORING_HIGH :
                shoulderGoal = coneMode ?
                    ArmConstants.SHOULDER_HIGH_CONE :
                    ArmConstants.SHOULDER_HIGH_CUBE;
                elbowGoal = coneMode ?
                    ArmConstants.ELBOW_HIGH_CONE :
                    ArmConstants.ELBOW_HIGH_CUBE;
                break;
            case SCORING_MID :
                shoulderGoal = coneMode ? ////
                        ArmConstants.SHOULDER_MID_CONE : ////
                        ArmConstants.SHOULDER_MID_CUBE;
                elbowGoal = coneMode ? //
                        ArmConstants.ELBOW_MID_CONE : ////
                        ArmConstants.ELBOW_MID_CUBE;
                break;  
        }
        shoulderStartPos = arm.shoulderEncoder.getAbsolutePosition();

        if (robot.arm.getDunkingCone()) {
            elbowGoal -= (20.0/360.0);
        }
    }

    @Override
    public void execute() {
        //Angle stuff
        if (gamePieceProtection && arm.getTrueForearmAngle().getSin() < 0) {
            arm.shoulderPIDController.setGoal(shoulderStartPos);
        } else {
            arm.shoulderPIDController.setGoal(shoulderGoal % 1);
        }
        arm.elbowPIDController.setGoal(elbowGoal % 1);
    }

    @Override
    public boolean isFinished() {
        return arm.shoulderPIDController.getGoal().position == (shoulderGoal % 1) && 
            (!yield || (arm.atShoulderGoal() && arm.atElbowGoal() && yield));
    }

    @Override
    public void end(boolean i) {
        arm.setCurrentPreset(set);
        SmartDashboard.putBoolean("SettingArm", false);
    }

    public SetArmPreset yield(){
        yield = true;
        return this;
    }


}
