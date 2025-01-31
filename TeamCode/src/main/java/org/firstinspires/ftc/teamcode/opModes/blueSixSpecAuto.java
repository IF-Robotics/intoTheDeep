package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@Autonomous(name="Blue 6+0\uD83D\uDC99\uD83D\uDC99\uD83D\uDC99\uD83E\uDD16")
public class blueSixSpecAuto extends sixSpecAuto{
    @Override
    public void initialize(){
        VisionSubsystem.alliance = VisionSubsystem.Alliance.BLUE;
        super.initialize();
    }
}
