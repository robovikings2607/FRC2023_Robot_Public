package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class RVProfiledPIDController extends ProfiledPIDController {
    TrapezoidProfile.Constraints constraints;
    public RVProfiledPIDController(double Kp, double Ki, double Kd, Constraints constraints) {
        super(Kp, Ki, Kd, constraints);
        this.constraints = constraints;
    }
    
    public RVProfiledPIDController(double Kp, double Ki, double Kd, Constraints constraints, double period) {
        super(Kp, Ki, Kd, constraints, period);
        this.constraints = constraints;
    }

    @Override
    public void setConstraints(TrapezoidProfile.Constraints constraints)
    {
        super.setConstraints(constraints);
        this.constraints = constraints;
    }

    public double totalTime()
    {
        return new TrapezoidProfile(constraints, getGoal(), getSetpoint()).totalTime();
    }
}
