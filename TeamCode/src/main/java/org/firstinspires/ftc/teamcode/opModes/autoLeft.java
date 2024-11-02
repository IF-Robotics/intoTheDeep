package org.firstinspires.ftc.teamcode.opModes;


import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPointCommand;

@TeleOp(name="autoLeft")
public class autoLeft extends Robot {

    @Override
    public void initialize(){
        super.initialize();

        schedule(new SequentialCommandGroup(
                new DriveToPointCommand(driveSubsystem, new SparkFunOTOS.Pose2D(0,20,0) ,0,0),
                new WaitCommand(200)
//                new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY),
                //intake



        ));

    }

}
