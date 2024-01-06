package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class AxisButton extends Button {
    public AxisButton(XboxController js, int axisID, double threshold)
    {
        super(() -> {
            return Math.abs(js.getRawAxis(axisID)) > threshold;
        });
    }
    
}
                                                                                                                                                   