package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;


@TeleOp(name="teleOp")
public class CommandTeleop extends CommandOpMode {

    public DcMotorEx BL, BR, FL, FR, arm, leftSlide, rightSlide;

    //subsystems
    public DriveSubsystem driveSubsystem;

    //commands
    private TeleDriveCommand teleDriveCommand;

    private GamepadEx m_driverOp;

    private LynxModule controlHub;

    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();
    double lastTime;

    @Override
    public void initialize() {
        //general
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        //hardware
        DcMotorEx BL, BR, FL, FR;
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);


        m_driverOp = new GamepadEx(gamepad1);

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL);
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }

    @Override
    public void run(){
        super.run();
        //clear cache
        controlHub.clearBulkCache();
        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();

    }
}
