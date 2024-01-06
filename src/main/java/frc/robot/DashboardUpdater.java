package frc.robot;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardUpdater<T> implements Runnable {

    Class<T> c;
    String prefix;
    Thread thread;

    public DashboardUpdater(Class<T> classInstance, String prefix)
    {
        this.prefix = prefix;
        this.c = classInstance;
        this.thread = new Thread(this);
        this.thread.start();
    }

    public String makeName(Field f)
    {
        return prefix + "/" + f.getName();
    }
    
    public void initField(Field f, String name)
    {
        try
        {
            if ( f.getType() == double.class )
            {
                SmartDashboard.putNumber(name, f.getDouble(null));
            }
            else if ( f.getType() == float.class )
            {
                SmartDashboard.putNumber(name, f.getFloat(null));
            }
            else if ( f.getType() == int.class )
            {
                SmartDashboard.putNumber(name, f.getInt(null));
            }
        }
        catch (Exception e)
        {
            System.err.println("Couldn't init " + f.getName());
            e.printStackTrace();
        }
    }

    public static void updateField(Field f, String name)
    {
        try
        {
            if ( f.getType() == double.class )
            {
                if ( !SmartDashboard.containsKey(name) )
                {
                    SmartDashboard.putNumber(name, f.getDouble(null));
                }
                f.setDouble(null, SmartDashboard.getNumber(name, f.getDouble(null)));
            }
            else if ( f.getType() == float.class )
            {
                if ( !SmartDashboard.containsKey(name) )
                {
                    SmartDashboard.putNumber(name, f.getFloat(null));
                }
                f.setFloat(null, (float)SmartDashboard.getNumber(name, f.getFloat(null)));
            }
            else if ( f.getType() == int.class )
            {
                if ( !SmartDashboard.containsKey(name) )
                {
                    SmartDashboard.putNumber(name, f.getInt(null));
                }
                f.setInt(null, (int)SmartDashboard.getNumber(name, f.getInt(null)));
            }
        }
        catch (Exception e)
        {
            System.err.println("Couldn't set " + f.getName());
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        var fields = c.getFields(); 
        for ( var field : fields )
        {
            final var modifiers = field.getModifiers();
            if ( (modifiers & Modifier.FINAL) == 0 && (modifiers & Modifier.STATIC) != 0 )
            {
                initField(field, makeName(field));
            }
        }
        while(true)
        {
            for ( var field : fields )
            {
                final var modifiers = field.getModifiers();
                if ( (modifiers & Modifier.FINAL) == 0 && (modifiers & Modifier.STATIC) != 0 )
                {
                    updateField(field, makeName(field));
                }
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
