package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@Autonomous(name="Red 6+0❤️❤️❤️\uD83E\uDD16")
public class redSixSpecAuto extends sixSpecAuto {
    @Override
    public void initialize() {
        VisionSubsystem.alliance = VisionSubsystem.Alliance.RED;
        super.initialize();
    }
}
