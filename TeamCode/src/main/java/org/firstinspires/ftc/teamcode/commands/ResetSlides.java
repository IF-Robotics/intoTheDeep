package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.manualSlides;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Globals;
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
        manualArm = true;
        manualSlides = true;
    }

    @Override
    public void execute(){
//        armSubsystem.setSlidePower(-0.1);
        armSubsystem.manualArm(0, -0.4);
        manualArm = true;
    }

    @Override
    public boolean isFinished(){
        return timer.seconds()>3.0 && armSubsystem.getSlideVelocity() < 0.25 && armSubsystem.getSlideExtention()<8.5;
//        return false;
    }

    @Override
    public void end(boolean e){
        armSubsystem.resetSlideEncoder();
        manualArm = false;
        manualSlides = false;
    }
}
