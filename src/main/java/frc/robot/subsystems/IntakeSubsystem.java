package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private ColorSensorV3 colorSensor;
    private Solenoid solenoid;
    private ColorMatch colorMatcher; //This is a dumb class
    private VictorSPX intakeMotor;

    private final Color CUBE_COLOR, CONE_COLOR;
    private ColorMatchResult seenColor;
    private boolean coneMode = true;
    private DigitalInput hasPieceSensor;

    private Counter intakeSensorHealth = new Counter(IntakeConstants.HAS_PIECE_SENSOR_HEALTHY);

    private GamePieceType currentGamePieceType = GamePieceType.NONE;

    public enum GamePieceType {
        CUBE,
        CONE,
        NONE
    }

    public IntakeSubsystem(RobotContainer robot) {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        intakeMotor = new VictorSPX(IntakeConstants.INTAKE_MOTOR_ID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(IntakeConstants.INTAKE_INVERTED);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        solenoid = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_SOLENOID);

        CUBE_COLOR = new Color(115, 0, 230);
        CONE_COLOR = new Color(255, 213, 0);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(CUBE_COLOR);
        colorMatcher.addColorMatch(CONE_COLOR);

        hasPieceSensor = new DigitalInput(IntakeConstants.HAS_PIECE_SENSOR);
    }

    public boolean intakeSensorHealthy()
    {
        return intakeSensorHealth.getPeriod() < 0.2;
    }

    public boolean hasPiece() {
        if ( !intakeSensorHealthy() )
        {
            return false;
        }
        return !hasPieceSensor.get();
    }
    
    public void setSolenoid(boolean state){
       solenoid.set(state);
    }

    public void setConeMode(boolean coneMode){
        this.coneMode = coneMode;
    }

    public void setMotor(double value){
        intakeMotor.set(ControlMode.PercentOutput, value);
    }

    public boolean getConeMode(){
        return coneMode;
    }

    /**
     * @return The game piece we're holding: Cube, Cone, or None
     */
    public GamePieceType getCurrentGamePiece() {
        return currentGamePieceType;
    }

    /**
     * @return Gamepiece proximity: ~40 is  far away, ~100 is holding, 2047 is against the sensor
     */
    public double getGamePieceProximity(){
        return colorSensor.getProximity();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("hasPiece", hasPiece());
        SmartDashboard.putBoolean("intakeSensorHealthy", intakeSensorHealthy());
        /* 
        SmartDashboard.putNumber("SensorR", colorSensor.getRed());
        SmartDashboard.putNumber("SensorB", colorSensor.getBlue());
        SmartDashboard.putNumber("SensorG", colorSensor.getGreen());
        SmartDashboard.putNumber("sensorDistance", getGamePieceProximity());
        */
        SmartDashboard.putBoolean("ConeMode?", coneMode);

        // seenColor = colorMatcher.matchClosestColor(colorSensor.getColor());


        //Below handles the current gamepiece in the intake
        // if (!(getGamePieceProximity() >= 100)) {
        //     currentGamePieceType = GamePieceType.NONE;
        //     return;
        // }
        // currentGamePieceType = seenColor.color.equals(CONE_COLOR) ? 
        //     GamePieceType.CONE :
        //     GamePieceType.CUBE ;
    }

}