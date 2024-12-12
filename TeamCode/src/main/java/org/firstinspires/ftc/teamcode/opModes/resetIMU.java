package org.firstinspires.ftc.teamcode.opModes;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.other.Robot;

@TeleOp(name="resetIMU")
public class resetIMU extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        CommandScheduler.getInstance().schedule(new InstantCommand(() -> driveSubsystem.resetPinpointIMU()));

    }



}