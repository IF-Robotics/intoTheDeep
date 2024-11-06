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

    private double timeInTolerance; //in milliseconds
    private ElapsedTime time = new ElapsedTime();

    public DriveToPointCommand(DriveSubsystem driveSubsystem, Pose2d targetPos, double translationalTolerance,double headingTolerance, double timeInTolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetPos = targetPos;
        this.translationalTolerance = translationalTolerance;
        this.headingTolerance = headingTolerance;
        this.timeInTolerance = timeInTolerance;
    }



    @Override
    public void execute() {
        driveSubsystem.readOtos();
        driveSubsystem.driveToPoint(targetPos);
    }

    @Override
    public boolean isFinished() {

        //if not in tolerance, then timer reset
        //if in tolerance and the timer matured enough, then finished
        //else not finished
        if(  driveSubsystem.getTranslationalError() > translationalTolerance && driveSubsystem.getHeadingError() > headingTolerance){
            time.reset();
            return false;
        } else if(time.milliseconds() > timeInTolerance){
            return true;
        } else {
            return false;
        }

        //this makes it so that the command will only finish when it reaches the target position and stays in the target position for a certain amount of time
    }
}
