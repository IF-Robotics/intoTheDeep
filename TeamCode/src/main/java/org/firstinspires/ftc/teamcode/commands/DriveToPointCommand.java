package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class DriveToPointCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Pose2d targetPos;
    private double translationalTolerance;
    private double headingTolerance;

    private ElapsedTime timer = new ElapsedTime();

    public DriveToPointCommand(DriveSubsystem driveSubsystem, Pose2d targetPos, double translationalTolerance,double headingTolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetPos = targetPos;
        this.translationalTolerance = translationalTolerance;
        this.headingTolerance = headingTolerance;
    }


    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        driveSubsystem.driveToPoint(targetPos);
    }

    @Override
    public boolean isFinished() {

        //if not in tolerance, then timer reset
        //if in tolerance and the timer matured enough, then finished
        //else not finished
        if((Math.abs(driveSubsystem.getTranslationalError()) > translationalTolerance || Math.abs(driveSubsystem.getHeadingError()) > headingTolerance) || timer.milliseconds() < 50){
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        //hold position
        //driveSubsystem.toggleAutoDrive(true);
    }
}
