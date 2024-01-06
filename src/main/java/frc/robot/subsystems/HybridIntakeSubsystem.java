package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class HybridIntakeSubsystem extends SubsystemBase {
    TalonSRX hybridMotor1;//, hybridMotor2;
    Solenoid solenoid;

    public HybridIntakeSubsystem(){
        hybridMotor1 = new TalonSRX(IntakeConstants.HYBRID_MOTOR_1_ID);
        // hybridMotor2 = new TalonSRX(IntakeConstants.HYBRID_MOTOR_2_ID);
        hybridMotor1.configFactoryDefault();
        // hybridMotor2.configFactoryDefault();
        hybridMotor1.setNeutralMode(NeutralMode.Brake);
        // hybridMotor2.setNeutralMode(NeutralMode.Brake);
        // hybridMotor2.follow(hybridMotor1);

        solenoid = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.HYBRID_SOLENOID);
    }

    public boolean getHybridIntakeState(){
        return solenoid.get();
    }

    public void setHybridIntake(double speed){
        hybridMotor1.set(ControlMode.PercentOutput, speed);
    }

    public void setHybridSolenoidState(boolean state){
        solenoid.set(state);
    }


    @Override
    public void periodic(){
        // SmartDashboard.putNumber("HybridCurrent", hybridMotor1.getStatorCurrent());
    }
}