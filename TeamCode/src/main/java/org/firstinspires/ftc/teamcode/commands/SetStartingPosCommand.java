package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class SetStartingPosCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Pose2d pos;

    public SetStartingPosCommand(DriveSubsystem driveSubsystem, Pose2d pos) {
        this.pos = pos;
        this.driveSubsystem = driveSubsystem;
//        addRequirements(driveSubsystem);


    }
    @Override
    public void execute() {
        driveSubsystem.setStartingPos(pos);

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
