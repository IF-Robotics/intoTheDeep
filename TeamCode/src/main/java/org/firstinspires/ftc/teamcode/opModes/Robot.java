package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.other.Globals.armBackX;
import static org.firstinspires.ftc.teamcode.other.Globals.armBackY;
import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armCloseIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighBasketY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHighChamberY;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armHomeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.intakeHoldPower;
import static org.firstinspires.ftc.teamcode.other.Globals.intakePower;
import static org.firstinspires.ftc.teamcode.other.Globals.manualArm;
import static org.firstinspires.ftc.teamcode.other.Globals.outtakePower;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenBasket;
import static org.firstinspires.ftc.teamcode.other.Globals.pitchWhenHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenArmBack;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenArmHome;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenCloseIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenHighChamber;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenReadyIntake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;

public abstract class Robot extends CommandOpMode {

    //commands
    public TeleDriveCommand teleDriveCommand;
    public ArmCommand armCommand;
    public SlideCommand slideCommand;
    public ArmCoordinatesCommand armCoordinatesCommand;
    public ArmManualCommand armManualCommand;
    public ArmCoordinatesCommand armHomeCommand;
    public ArmCoordinatesCommand armHighBasketCommand;
    public ArmCoordinatesCommand armBackCommand;
    public ArmCoordinatesCommand armWhenIntakeCommand;
    public ArmCoordinatesCommand armWhenCloseIntakeCommand;
    public ArmCoordinatesCommand armWhenHighChamberCommand;
    public IntakeCommand setIntakeCommand;
    public IntakeCommand intakeWhenArmBackCommand;
    public IntakeCommand intakeWhenHighBasketCommand;
    public IntakeCommand outakeWhenHighBasketCommand;
    public IntakeCommand intakeReadyCommand;
    public IntakeCommand outakeReadyCommand;
    public IntakeCommand intakeWhenArmHomeCommand;
    public IntakeCommand intakeCommand;
    public IntakeCommand intakeWhenHighChamberCommand;
    public IntakeCommand intakeCloseCommand;

    //test statics
    public static double x = 0, y = 0;
    public static double pitch = 0, roll = 0;

    //hardware
    public MotorEx BL, BR, FL, FR, arm, slideLeft, slideRight;
    public MotorGroup slide;
    public CRServo intake;
    public ServoEx diffyLeft, diffyRight;
    public AnalogInput armEncoder;
    public SparkFunOTOS myOtos;

    //subsystems
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;

    //system
    private LynxModule controlHub;
    //gamePads
    public GamepadEx m_driver;
    public GamepadEx m_driverOp;

    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();

    boolean manual = false;
    boolean flag = false;

    public void initialize(){
        time.reset();

        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Otos
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

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

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL, telemetry, myOtos);
        register(driveSubsystem);

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
        register(armSubsystem);
        //armSubsystem.setDefaultCommand(armHomeCommand);

        //intake
        intake = new CRServo(hardwareMap, "intake");
        diffyLeft =  new SimpleServo(hardwareMap, "diffyLeft", 0, 360, AngleUnit.DEGREES);
        diffyRight =  new SimpleServo(hardwareMap, "diffyRight", 0, 360, AngleUnit.DEGREES);
        intake.setInverted(true);

        intakeSubsystem = new IntakeSubsystem(intake, diffyLeft, diffyRight, telemetry);
        register(intakeSubsystem);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        configureCommands();

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

        //clear cache
        //other telemetry
        telemetry.addData("manual", manualArm);
        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();
        controlHub.clearBulkCache();

    }

    public void configureCommands(){
        teleDriveCommand = new TeleDriveCommand(driveSubsystem, m_driver, true, 10, m_driver::getLeftX, m_driver::getLeftY, m_driver::getRightX);

        //ARM
        armCommand = new ArmCommand(armSubsystem, m_driverOp::getLeftY);
        slideCommand = new SlideCommand(armSubsystem, m_driverOp::getRightY);
        armCoordinatesCommand = new ArmCoordinatesCommand(armSubsystem, x, y);
        //home poses
        armHomeCommand = new ArmCoordinatesCommand(armSubsystem, armHomeX, armHomeY);
        armBackCommand = new ArmCoordinatesCommand(armSubsystem, armBackX, armBackY);
        //scoring
        armHighBasketCommand = new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY);
        armWhenHighChamberCommand = new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY);
        //intaking
        armWhenIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armIntakeX, armIntakeY);
        armWhenCloseIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY);


        armManualCommand = new ArmManualCommand(armSubsystem, m_driverOp, m_driverOp::getRightY, m_driverOp::getLeftY);

        //INTAKE
        setIntakeCommand = new IntakeCommand(intakeSubsystem, 0, pitch, roll);

        //scoring
        intakeWhenHighBasketCommand = new IntakeCommand(intakeSubsystem, 0, pitchWhenBasket, 0);
        intakeWhenHighChamberCommand = new IntakeCommand(intakeSubsystem, intakeHoldPower, pitchWhenHighChamber, rollWhenHighChamber);

        //intaking
        intakeReadyCommand = new IntakeCommand(intakeSubsystem, intakeHoldPower, 0, rollWhenReadyIntake);
        outakeReadyCommand = new IntakeCommand(intakeSubsystem, outtakePower, 0, rollWhenReadyIntake);
        intakeCloseCommand = new IntakeCommand(intakeSubsystem, intakePower, 0, rollWhenCloseIntake);

        //home poses
        intakeWhenArmHomeCommand = new IntakeCommand(intakeSubsystem, 0, 0, rollWhenArmHome);
        intakeWhenArmBackCommand = new IntakeCommand(intakeSubsystem, intakePower, pitchWhenBasket, rollWhenArmBack);
        intakeCommand = new IntakeCommand(intakeSubsystem, intakePower, 0, rollWhenIntake);

    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);


        //Might not always want to do
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}