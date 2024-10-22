package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmReset extends CommandBase {
    private ArmSubsystem armSubsystem;
    private MotorEx arm;
    private MotorEx slideL;
    private MotorGroup slides;
    private ElapsedTime time = new ElapsedTime();

    public ArmReset(ArmSubsystem armSubsystem, MotorEx arm, MotorEx slideL, MotorGroup slides) {
        this.armSubsystem = armSubsystem;
        this.arm = arm;
        this.slideL = slideL;
        this.slides = slides;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        time.reset();
        arm.set(-.1);
        slides.set(-.2);
    }

    @Override
    public boolean isFinished() {
        if (time.seconds() > 1) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopAndResetEncoder();
        slideL.stopAndResetEncoder();
        arm.set(0);
        slides.set(0);
    }
}
