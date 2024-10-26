package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;


@Config
@TeleOp(name="teleOp")
public class CommandTeleop extends CommandOpMode {

    public static double x = 0, y = 0;
    public static double pitch = 0, roll = 0;

    //hardware
    public MotorEx BL, BR, FL, FR, arm, slideLeft, slideRight;
    public MotorGroup slide;
    public CRServo intake;
    public ServoEx diffyLeft, diffyRight;
    public AnalogInput armEncoder;

    //subsystems
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;

    //commands
    private TeleDriveCommand teleDriveCommand;
    private ArmCommand armCommand;
    private SlideCommand slideCommand;
    private ArmCoordinatesCommand armCoordinatesCommand;
    private ArmManualCommand armManualCommand;
    private ArmCoordinatesCommand armHomeCommand;
    private ArmCoordinatesCommand armHighBasketCommand;
    private ArmCoordinatesCommand armBackCommand;
    private ArmCoordinatesCommand armWhenIntakeCommand;
    private IntakeCommand setIntakeCommand;
    private IntakeCommand intakeWhenArmBackCommand;
    private IntakeCommand intakeWhenHighBasketCommand;
    private IntakeCommand outakeWhenHighBasketCommand;
    private IntakeCommand intakeReadyCommand;
    private IntakeCommand intakeWhenArmHomeCommand;
    private IntakeCommand intakeCommand;

    //buttons
    private Button x1, back2, start2, dUp1, dDown1, dLeft1, dRight1, bRight1, bLeft1, triangle1;
    private Trigger tLeft1, tRight1;

    //gamePads
    private GamepadEx m_driver;
    private GamepadEx m_driverOp;

    //system
    private LynxModule controlHub;



    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();

    boolean manual = false;
    boolean flag = false;

    //constants
    public static int rollWhenReadyIntake = 150;
    public static int rollWhenIntake = 80;



    @Override
    public void initialize() {
        time.reset();

        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //dt
        FL = new MotorEx(hardwareMap, "FL");
        FR = new MotorEx(hardwareMap, "FR");
        BL = new MotorEx(hardwareMap, "BL");
        BR = new MotorEx(hardwareMap, "BR");
        FL.setRunMode(MotorEx.RunMode.RawPower);
        FR.setRunMode(MotorEx.RunMode.RawPower);
        BL.setRunMode(MotorEx.RunMode.RawPower);
        BR.setRunMode(MotorEx.RunMode.RawPower);
        FR.setInverted(true);
        BR.setInverted(true);

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL);
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(teleDriveCommand);

        //arm
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_30);
        slideLeft = new MotorEx(hardwareMap, "slideL");
        slideRight = new MotorEx(hardwareMap, "slideR");
        armEncoder = hardwareMap.get(AnalogInput.class, "armEncoder");
        arm.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setRunMode(Motor.RunMode.RawPower);
        slideRight.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideLeft.setInverted(false);
        slideRight.setInverted(true);
        arm.setInverted(false);

        slideLeft.resetEncoder();

        slide = new MotorGroup(slideLeft, slideRight);

        armSubsystem = new ArmSubsystem(arm, slideLeft, slide, diffyLeft, diffyRight, armEncoder, telemetry);
        armCommand = new ArmCommand(armSubsystem, m_driverOp::getLeftY);
        slideCommand = new SlideCommand(armSubsystem, m_driverOp::getRightY);
        armCoordinatesCommand = new ArmCoordinatesCommand(armSubsystem, x, y);
        armHomeCommand = new ArmCoordinatesCommand(armSubsystem, 7.2, 5);
        armBackCommand = new ArmCoordinatesCommand(armSubsystem, -3, 15);
        armHighBasketCommand = new ArmCoordinatesCommand(armSubsystem, -8, 43);
        armWhenIntakeCommand = new ArmCoordinatesCommand(armSubsystem, 20, 3.75);
        armManualCommand = new ArmManualCommand(armSubsystem, m_driverOp::getRightY, m_driverOp::getLeftY);

        register(armSubsystem);
        //armSubsystem.setDefaultCommand(armHomeCommand);

        //intake
        intake = new CRServo(hardwareMap, "intake");
        diffyLeft =  new SimpleServo(hardwareMap, "diffyLeft", 0, 360, AngleUnit.DEGREES);
        diffyRight =  new SimpleServo(hardwareMap, "diffyRight", 0, 360, AngleUnit.DEGREES);
        intake.setInverted(true);

        intakeSubsystem = new IntakeSubsystem(intake, diffyLeft, diffyRight, telemetry);
        setIntakeCommand = new IntakeCommand(intakeSubsystem, 0, pitch, roll);
        intakeWhenArmBackCommand = new IntakeCommand(intakeSubsystem, .1, 240, -150);

        intakeWhenHighBasketCommand = new IntakeCommand(intakeSubsystem, 0, 240, 0);
        outakeWhenHighBasketCommand = new IntakeCommand(intakeSubsystem, -.5, 240, 0);

        intakeReadyCommand = new IntakeCommand(intakeSubsystem, 1, 0, rollWhenReadyIntake);
        intakeWhenArmHomeCommand = new IntakeCommand(intakeSubsystem, 0, 0, 300);
        intakeCommand = new IntakeCommand(intakeSubsystem, 1, 0, rollWhenIntake);

        register(intakeSubsystem);


        configureButtons();


        bLeft1.whenPressed(armWhenIntakeCommand);
        bLeft1.whenPressed(intakeReadyCommand);
        bLeft1.whenReleased(intakeCommand);

        dDown1.whenPressed(armHomeCommand);

        //baskets
        triangle1.whenPressed(armHighBasketCommand);
        triangle1.whenPressed(intakeWhenHighBasketCommand);
        x1.whenPressed(outakeWhenHighBasketCommand);

        dLeft1.whenPressed(armBackCommand);
        dLeft1.whenPressed(intakeWhenArmBackCommand);

        //climbing
        start2.whenPressed(intakeWhenHighBasketCommand);



        telemetry.addLine("Initialized");
        telemetry.update();
    }



    @Override
    public void run(){
        /*if (!flag) {
            super.run(); //whatever you need to run once
            flag = true;
        }*/

        super.run();

        //arm input testing
        //armCoordinatesCommand = new ArmCoordinatesCommand(armSubsystem, x, y);
        /*if(gamepad2.cross){
            armCoordinatesCommand.schedule();
        }*/

        //intake input testing
        //setIntakeCommand = new IntakeCommand(intakeSubsystem, 1, pitch, roll);
        /*if(gamepad2.circle) {
            setIntakeCommand.schedule();
        }*/

        if(manual){
            armManualCommand.schedule();
            intakeWhenHighBasketCommand.schedule();
        }

        //manualMode
        if(gamepad2.start){
            manual = true;
        } else if(gamepad2.back){
            manual = false;
        }



        //clear cache
        //other telemetry
        telemetry.addData("manual", manual);
        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();
        controlHub.clearBulkCache();

    }

    public void configureButtons() {
        x1 = new GamepadButton(m_driver, GamepadKeys.Button.X);
        start2 = new GamepadButton(m_driverOp, GamepadKeys.Button.START);
        back2 = new GamepadButton(m_driverOp, GamepadKeys.Button.BACK);
        dUp1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_UP);
        dDown1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_DOWN);
        dLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_LEFT);
        dRight1 = new GamepadButton(m_driver, GamepadKeys.Button.DPAD_RIGHT);
        bRight1 = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER);
        bLeft1 = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER);
        triangle1 = new GamepadButton(m_driver, GamepadKeys.Button.Y);
    }
}
