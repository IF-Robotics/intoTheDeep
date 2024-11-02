package org.firstinspires.ftc.teamcode.opModes;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;

@TeleOp(name="autoTest")
public class autoTest extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new DriveToPointCommand(driveSubsystem, SparkFunOTOS.Pose2D(0,0,0) ,0,0)
        );

    }

}
