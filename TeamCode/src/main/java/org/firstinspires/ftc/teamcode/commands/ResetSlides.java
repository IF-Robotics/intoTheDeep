package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.BooleanSupplier;

public class ResetSlides extends CommandBase {

    ArmSubsystem armSubsystem;
    ElapsedTime timer = new ElapsedTime();
    private boolean timerConditionSatisfied = false;
    private boolean hasReset = false;
    public ResetSlides(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;


        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        if (armSubsystem.getSlideVelocity() < 0.5 && (timer.seconds()>0.5)||timerConditionSatisfied){
            if(timerConditionSatisfied==false){
                timer.reset();
            }
            timerConditionSatisfied = true;
            if (timer.seconds()>1.0 && timerConditionSatisfied){
                hasReset=true;
                armSubsystem.resetSlideEncoder();
            }
        }

//        armSubsystem.setSlidePower(-0.1);
        armSubsystem.manualArm(0, -0.25);
        manualArm = true;
    }

    @Override
    public boolean isFinished(){
        return hasReset;
//        return false;
    }

    @Override
    public void end(boolean e){
        manualArm = false;
    }
}
