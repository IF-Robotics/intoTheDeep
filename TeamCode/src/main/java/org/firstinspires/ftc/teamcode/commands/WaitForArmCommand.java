package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

public class WaitForArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double targetAngle;
    private double tolerance;
    private ElapsedTime timer = new ElapsedTime();
    private int loopCount = 0;

    public WaitForArmCommand(ArmSubsystem armSubsystem, double targetAngle, double tolerance) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;


        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArm(targetAngle);
        Log.i("stupidbruhtargetAngle", String.valueOf(targetAngle));
        timer.reset();
        loopCount = 0;
    }

    @Override
    public void execute(){
        armSubsystem.setArm(targetAngle);
        Log.i("stupidbruhtargetAngle", String.valueOf(targetAngle));
        loopCount++;
    }

    @Override
    public boolean isFinished(){
        if(((armSubsystem.getArmAngle() > targetAngle - tolerance) && (armSubsystem.getArmAngle() < targetAngle + tolerance)) && loopCount >= 2){
            return true;
        } else {
            return false;
        }
    }

}
