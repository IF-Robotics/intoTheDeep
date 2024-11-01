package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
public class DriveToPointCommand extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private double targetx;
    private double targety;
    private double targetheading;

    public DriveToPointCommand(double targetx, double targety, double targetheading){
        this.targetx = targetx;
        this.targety = targety;
        this.targetheading = targetheading;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.driveToPoint(targetx, targety, targetheading);

    }
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
}
