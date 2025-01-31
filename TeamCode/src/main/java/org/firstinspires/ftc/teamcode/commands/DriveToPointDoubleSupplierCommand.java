package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveToPointDoubleSupplierCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private double translationalTolerance;
    private double headingTolerance;

    private DoubleSupplier x;
    private DoubleSupplier y;
    private Rotation2d rotation;

    private ElapsedTime timer = new ElapsedTime();
    private int loopCount = 0;

    public DriveToPointDoubleSupplierCommand(DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, Rotation2d rotation, double translationalTolerance, double headingTolerance) {
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y=y;
        this.rotation = rotation;
        this.translationalTolerance = translationalTolerance;
        this.headingTolerance = headingTolerance;

        addRequirements();
    }


    @Override
    public void initialize() {
        timer.reset();
        loopCount = 0;
        Pose2d targetPos = new Pose2d(x.getAsDouble(), y.getAsDouble(), rotation);
        Log.i("bruhx", String.valueOf(x.getAsDouble()-3));
        Log.i("bruhy", String.valueOf(y.getAsDouble()+46.5));
        driveSubsystem.driveToPoint(targetPos);
    }

    @Override
    public void execute() {
        loopCount++;
    }

    @Override
    public boolean isFinished() {

        //if not in tolerance, then timer reset
        //if in tolerance and the timer matured enough, then finished
        //else not finished
        if((Math.abs(driveSubsystem.getTranslationalError()) > translationalTolerance || Math.abs(driveSubsystem.getHeadingError()) > headingTolerance) || loopCount <= 1){
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
