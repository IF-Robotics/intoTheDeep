package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

public class DriveToPointCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private SparkFunOTOS.Pose2D targetPos;
    private double translationalTolerance;

    private double timeInTolerance; //in milliseconds
    private ElapsedTime time;

    public DriveToPointCommand(DriveSubsystem driveSubsystem, SparkFunOTOS.Pose2D targetPos, double translationalTolerance, double timeInTolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetPos = targetPos;
    }

    @Override
    public void initialize() {
        time.reset();
    }

    @Override
    public void execute() {
        driveSubsystem.driveToPoint(targetPos);
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
