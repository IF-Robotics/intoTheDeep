package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    DriveSubsystem driveSubsystem;

    public AutoDriveCommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        //addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.autoDrive();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
