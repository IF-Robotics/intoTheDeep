package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem.alliance;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.VisionClawCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.opModes.TeleopOpMode;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;
@TeleOp
public class blueTeleop extends TeleopOpMode {
    public void initialize(){
        super.initialize();
//        visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED); //They're opposites
//        alliance = VisionSubsystem.Alliance.RED;

    }


}
