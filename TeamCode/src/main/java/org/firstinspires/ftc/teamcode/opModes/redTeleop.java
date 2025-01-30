package org.firstinspires.ftc.teamcode.opModes;

//import static org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem.alliance;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.other.Globals.teleopSpec;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.commands.VisionClawCommand;
import org.firstinspires.ftc.teamcode.other.Robot;
import org.firstinspires.ftc.teamcode.opModes.TeleopOpMode;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

//import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;
@TeleOp(name="RedTeleop❤️❤️❤️")
public class redTeleop extends TeleopOpMode {
    public void initialize(){
        VisionSubsystem.alliance = VisionSubsystem.Alliance.RED;
        super.initialize();
    }

    @Override
    public void run(){
        super.run();

        if(teleopSpec){
            gamepad1.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
        } else{
            gamepad1.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
        }
    }


}
