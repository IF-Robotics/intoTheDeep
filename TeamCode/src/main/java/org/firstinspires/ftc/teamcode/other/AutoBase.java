package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;

public abstract class AutoBase extends Robot{

    public void initialize() {
        super.initialize();

        //reset slide encoder
        slideLeft.resetEncoder();

        //setdefault commands
        driveSubsystem.setDefaultCommand(new AutoDriveCommand(driveSubsystem));
    }

    public void configureAutoCommands(){

    }


}
