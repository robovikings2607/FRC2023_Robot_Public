package frc.robot.utilities;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class REVChecker {
    static boolean errorDetected = false;

    public static void check(REVLibError result, String description)
    {
        if ( result != REVLibError.kOk )
        {
            DataLogManager.log("REV error "+ description + ": "+ result);
            errorDetected = true;
        }
    }

    public static void checkFunction(Supplier<REVLibError> func, String description)
    {
        REVLibError result = null;
        for ( int i = 0; i < 3; i++)
        {
            result = func.get();
            if ( result == REVLibError.kOk )
            {
                DataLogManager.log("REV " +description+ ": Success");
                return;
            }
            DataLogManager.log("REV error "+ description + ": "+ result);  
            Timer.delay(0.1);
        }
        errorDetected = true;
    }

    public static boolean hadError()
    {
        return errorDetected;
    }
}
