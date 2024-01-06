package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPresets;
import frc.robot.subsystems.sim.ArmSimulation;
import frc.robot.utilities.REVChecker;

public class ArmSubsystem extends SubsystemBase{

    private CANSparkMax shoulderMainMotor, shoulderSlaveMotor, elbowMainMotor, elbowSlaveMotor;
    public DutyCycleEncoder shoulderEncoder, elbowEncoder;
    public ProfiledPIDController shoulderPIDController, elbowPIDController;
    private boolean dunkingCone;
    private ArmPresets currentPreset;

    private ArmSimulation armSimulation;

    Mechanism2d mechanism = new Mechanism2d(10, 10);
    MechanismRoot2d rootArm = mechanism.getRoot("Arm", 5, 5);
    MechanismLigament2d shoulder2d = rootArm.append(new MechanismLigament2d("shoulder", 3, 0));
    MechanismLigament2d elbow2d = shoulder2d.append(new MechanismLigament2d("elbow", 1.5, 0));


    public ArmSubsystem() {
        setDunkingCone(false);

        shoulderEncoder = new DutyCycleEncoder(ArmConstants.SHOULDER_ENCODER_PORT);
        elbowEncoder = new DutyCycleEncoder(ArmConstants.ELBOW_ENCODER_PORT);
        
        //Construct shoulder motors
        shoulderMainMotor = new CANSparkMax(ArmConstants.SHOULDER_MASTER, MotorType.kBrushless);
        shoulderSlaveMotor = new CANSparkMax(ArmConstants.SHOULDER_SLAVE, MotorType.kBrushless);
        REVChecker.check(shoulderMainMotor.restoreFactoryDefaults(), "shoulder main restore defaults");
        REVChecker.check(shoulderSlaveMotor.restoreFactoryDefaults(), "shoulder slave restore defaults");
        REVChecker.checkFunction(() -> shoulderSlaveMotor.follow(shoulderMainMotor, true), "shoulder.follow");
        shoulderSlaveMotor.setControlFramePeriodMs(0);

        //Construct elbow stuff
        elbowMainMotor = new CANSparkMax(ArmConstants.ELBOW_MASTER, MotorType.kBrushless);
        elbowSlaveMotor = new CANSparkMax(ArmConstants.ELBOW_SLAVE, MotorType.kBrushless);
        elbowMainMotor.restoreFactoryDefaults();
        elbowSlaveMotor.restoreFactoryDefaults();
        REVChecker.checkFunction(() -> {
            elbowMainMotor.setInverted(true);
            return elbowMainMotor.getLastError();
        }, "elbow.setInverted");
        REVChecker.checkFunction(() -> elbowSlaveMotor.follow(elbowMainMotor, true), "elbow.follow");
        elbowSlaveMotor.setControlFramePeriodMs(0);

        //shoulder PID
        shoulderPIDController = new ProfiledPIDController( //Profiled PID
            ArmConstants.SHOULDER_P,
            0,
            ArmConstants.SHOULDER_D,
            new Constraints(0.7, 0.7)
        );
        shoulderPIDController.setGoal(ArmConstants.SHOULDER_STOWING);
        shoulderPIDController.enableContinuousInput(0, 1);
        shoulderPIDController.setTolerance(8.0/360.0);

        //elbow PID
        elbowPIDController = new ProfiledPIDController(ArmConstants.ELBOW_P, 
            0, 
            ArmConstants.ELBOW_D, 
            new Constraints(0.7,0.7)
        );
        elbowPIDController.setGoal(ArmConstants.ELBOW_STOWING);
        elbowPIDController.enableContinuousInput(0, 1);
        elbowPIDController.setTolerance(0.01);

        //Dashboard tuning puts
        // SmartDashboard.putNumber("neutralElbowPositionLALALA", ArmConstants.ELBOW_NEUTRAL);
        // SmartDashboard.putNumber("ShoulderNeutral", ArmConstants.SHOULDER_NEUTRAL);
        
        // SmartDashboard.putNumber("ShoulderVelocityFF", ArmConstants.SHOULDER_VELOCITY_FF);
        // SmartDashboard.putNumber("ElbowVelocityFF", ArmConstants.ELBOW_VELOCITY_FF);

        // SmartDashboard.putNumber("ShoulderP", ArmConstants.SHOULDER_P);
        // SmartDashboard.putNumber("ElbowP", ArmConstants.ELBOW_P);

        if (Robot.isSimulation())
        {
            armSimulation = new ArmSimulation(this, shoulderPIDController, shoulderEncoder, elbowPIDController, elbowEncoder);
        }

        SmartDashboard.putData("ArmMechanism", mechanism);
    }


    public void setDunkingCone(boolean dunking) {
        dunkingCone = dunking;
        SmartDashboard.putBoolean("Dunking Cone", dunkingCone);
    }

    /**
     * Sets the velocity of the master and slave motors, capped at +- max velocity 
     */
    public void setShoulderOpenLoop(double input) {
        SmartDashboard.putNumber("ShoulderOutput", Constants.Clamp(-1, 1, input));
        shoulderMainMotor.set(Constants.Clamp(-1, 1, input));
    }

    public void setElbowOpenLoop(double input) {
        SmartDashboard.putNumber("ElbowOutput", Constants.Clamp(-1, 1, input));
        elbowMainMotor.set(Constants.Clamp(-1, 1, input));
    }

    public void setCurrentPreset(ArmPresets preset) {
        currentPreset = preset;
    }

    public boolean getDunkingCone() {
        return dunkingCone;
    }

    public ArmPresets getCurrentPreset() {
        return currentPreset;
    }

    public boolean atShoulderGoal(){
        return shoulderPIDController.atGoal();
    }

    public boolean atElbowGoal(){
        return elbowPIDController.atGoal();
    }

    /**
     * @return The angle between the floor and the shoulder as a Rotation2d. Keep in mind
     * the fact that the encoder reads 0-1, and the reading where the shoulder extends straight
     * down is ArmConstants.SHOULDER_ZERO
     */
    public Rotation2d getTrueShoulderAngle() {
        return Rotation2d.fromRotations(
            (ArmConstants.SHOULDER_ZERO - shoulderEncoder.getAbsolutePosition()) - (90.0/360.0)
        );
    }
    /**
     * @return The angle between the floor and the forearm as a Rotation2d. Keep in mind
     * the fact that the encoder reads 0-1, and the reading where the forearm extends parallel to the
     * shoulder arm is ArmConstants.ELBOW_ZERO
     */
    public Rotation2d getTrueForearmAngle() {
        return Rotation2d.fromRotations(
            getTrueShoulderAngle().getRotations() +
            (elbowEncoder.getAbsolutePosition() - ArmConstants.ELBOW_ZERO)
        );
    }
    
    public Rotation2d getForearmRelativeToShoulder()
    {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - ArmConstants.ELBOW_ZERO);
    }

    double elbowPIDCalculate()
    {
        return elbowPIDController.calculate(elbowEncoder.getAbsolutePosition());
    }

    double shoulderPIDCalculate()
    {
        return shoulderPIDController.calculate(shoulderEncoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ABSShoulderEncoder", shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ShoulderPIDGoal", shoulderPIDController.getSetpoint().position);
        SmartDashboard.putBoolean("ShoulderAtGoal", atShoulderGoal());

        SmartDashboard.putNumber("ABSElbowEncoder", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ElbowPIDGoal", elbowPIDController.getSetpoint().position);
        SmartDashboard.putBoolean("ElbowAtGoal", atElbowGoal());

        SmartDashboard.putBoolean("ShoulderEncoderConnected", shoulderEncoder.isConnected());
        SmartDashboard.putBoolean("ElbowEncoderConnected", elbowEncoder.isConnected());

        boolean armWorking = shoulderEncoder.isConnected() && elbowEncoder.isConnected();
        SmartDashboard.putBoolean("ArmBroken", !armWorking);
        
        if (shoulderEncoder.isConnected() && elbowEncoder.isConnected())
        {
            setElbowOpenLoop(
                elbowPIDCalculate() +
                (elbowPIDController.getSetpoint().velocity * ArmConstants.ELBOW_VELOCITY_FF) +
                (getTrueForearmAngle().getCos() * ArmConstants.ELBOW_NEUTRAL)
            );
    
            setShoulderOpenLoop(
                shoulderPIDCalculate() +
                (shoulderPIDController.getSetpoint().velocity * ArmConstants.SHOULDER_VELOCITY_FF) +
                (getTrueShoulderAngle().getCos() * ArmConstants.SHOULDER_NEUTRAL)
            );
        } else {
            setElbowOpenLoop(0);
            setShoulderOpenLoop(0);
        }
        

        shoulder2d.setAngle(getTrueShoulderAngle());
        elbow2d.setAngle(getForearmRelativeToShoulder());
    }

    public void simulationPeriodic()
    {
        armSimulation.simulationPeriodic();
    }
}