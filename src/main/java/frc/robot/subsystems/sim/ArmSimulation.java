package frc.robot.subsystems.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSimulation {

    ArmSubsystem arm;

    ProfiledPIDController shoulderController, elbowController;
    DutyCycleEncoder shoulderEncoder, elbowEncoder;
    FixedDutyCycleEncoderSim shoulderSim, elbowSim;



    public ArmSimulation(ArmSubsystem arm, ProfiledPIDController shoulderController, DutyCycleEncoder shoulderEncoder, ProfiledPIDController elbowController, DutyCycleEncoder elbowEncoder)
    {
        this.arm = arm;

        this.shoulderController = shoulderController;
        this.shoulderEncoder = shoulderEncoder;

        this.elbowController = elbowController;
        this.elbowEncoder = elbowEncoder;

        shoulderSim = new FixedDutyCycleEncoderSim(shoulderEncoder);
        elbowSim = new FixedDutyCycleEncoderSim(elbowEncoder);
    }
    
    public void simulationPeriodic()
    {
        shoulderSim.setAbsolutePosition(MathUtil.inputModulus(shoulderController.getSetpoint().position, 0.0, 1.0));
        elbowSim.setAbsolutePosition(MathUtil.inputModulus(elbowController.getSetpoint().position, 0.0, 1.0));


    }
}
