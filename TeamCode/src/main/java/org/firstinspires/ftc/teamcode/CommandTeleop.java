package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;


@TeleOp(name="teleOp")
public class CommandTeleop extends CommandOpMode {

    public MotorEx BL, BR, FL, FR, arm, slideLeft, slideRight;
    public MotorGroup slide;
    public CRServo intake;
    public Servo diffyLeft, diffyRight;

    //subsystems
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;

    //commands
    private TeleDriveCommand teleDriveCommand;
    private ArmCommand armCommand;

    private GamepadEx m_driver;
    private GamepadEx m_driverOp;

    private LynxModule controlHub;

    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();

    @Override
    public void initialize() {
        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        //dt
        FL = new MotorEx(hardwareMap, "FL");
        FR = new MotorEx(hardwareMap, "FR");
        BL = new MotorEx(hardwareMap, "BL");
        BR = new MotorEx(hardwareMap, "BR");
        FL.setRunMode(MotorEx.RunMode.RawPower);
        FR.setRunMode(MotorEx.RunMode.RawPower);
        BL.setRunMode(MotorEx.RunMode.RawPower);
        BR.setRunMode(MotorEx.RunMode.RawPower);
        FL.setInverted(true);
        BL.setInverted(true);

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL);
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(teleDriveCommand);

        //arm
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_30);
        slideLeft = new MotorEx(hardwareMap, "slideL");
        slideRight = new MotorEx(hardwareMap, "slideR");
        arm.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setRunMode(Motor.RunMode.RawPower);
        slideRight.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideLeft.setInverted(true);
        arm.setInverted(true);
        slide = new MotorGroup(slideLeft, slideRight);

        armSubsystem = new ArmSubsystem(arm, slideLeft, telemetry);
        armCommand = new ArmCommand(armSubsystem, m_driverOp::getLeftY);
        register(armSubsystem);
        armSubsystem.setDefaultCommand(armCommand);

        //intake
        intake = new CRServo(hardwareMap, "intake");
        diffyLeft =  hardwareMap.get(Servo.class, "diffyLeft");
        diffyRight =  hardwareMap.get(Servo.class, "diffyRight");

        //register(intakeSubsystem);

        //other
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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
