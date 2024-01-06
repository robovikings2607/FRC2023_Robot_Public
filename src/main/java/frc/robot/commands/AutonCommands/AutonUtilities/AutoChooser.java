package frc.robot.commands.AutonCommands.AutonUtilities;

import java.io.Closeable;
import java.io.IOException;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser implements NTSendable, Closeable {
    SendableChooser<CommandBuilder> chooser = new SendableChooser<>();

    public void addOption(String name, CommandBuilder builder)
    {
        chooser.addOption(name, builder);
        if (builder != null)
        {
            builder.build();
        }
    }
    
    public void setDefaultOption(String name, CommandBuilder builder)
    {   
        chooser.setDefaultOption(name, builder);
        if (builder != null)
        {
            builder.build();
        }
    }

    public Command getSelected()
    {
        CommandBuilder builder = chooser.getSelected();
        if (builder == null)
        {
            return null;
        }
        return builder.build();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        chooser.initSendable(builder);
    }

    @Override
    public void close() throws IOException {
        chooser.close();
    }
}
