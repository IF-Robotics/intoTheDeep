package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class holdDTPosCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;

    public holdDTPosCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
//        driveSubsystem.driveToPoint(driveSubsystem.getTargetPos());
        driveSubsystem.autoDrive();
    }

}
