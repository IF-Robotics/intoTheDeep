package org.firstinspires.ftc.teamcode.other;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandGroups.DropOffCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighBasketCommand;
import org.firstinspires.ftc.teamcode.commandGroups.HighChamberCommand;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractAfterWallIntake;
import org.firstinspires.ftc.teamcode.commandGroups.RetractFromBasket;
import org.firstinspires.ftc.teamcode.commandGroups.ScoreHighChamberCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commandGroups.DropCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

@Config
public abstract class Robot extends CommandOpMode {

    //commands
    public static TeleDriveCommand teleDriveCommand;
    public static ArmCommand armCommand;
    public static SlideCommand slideCommand;
    public static ArmCoordinatesCommand armCoordinatesCommand;
    public static ArmManualCommand armManualCommand;
    public static ArmCoordinatesCommand armHomeCommand;
    public static ArmCoordinatesCommand armHighBasketCommand;
    public static ArmCoordinatesCommand armBackCommand;
    public static ArmCoordinatesCommand armInSubCommand;
    public static ArmCoordinatesCommand armWhenIntakeWallCommand;
    public static ArmCoordinatesCommand armWhenCloseIntakeCommand;
    public static ArmCoordinatesCommand armWhenHighChamberCommand;
    public static ArmCoordinatesCommand armFrontHighChamberCommand;
    public static ArmCoordinatesCommand armPositionToClimb;
    public static ArmCoordinatesCommand armLeftAutoParkCommand;
    public static ArmCoordinatesCommand armAutoRightCommand;
    public static IntakeCommand setIntakeCommand;
    public static IntakeCommand intakeWhenArmBackCommand;
    public static IntakeCommand intakeWhenHighBasketCommand;
    public static IntakeCommand outakeWhenHighBasketCommand;
    public static IntakeCommand outakeReadyCommand;
    public static IntakeCommand intakeWhenArmHomeCommand;
    public static IntakeCommand intakeCommand;
    public static IntakeCommand intakeWhenHighChamberCommand;
    public static IntakeCommand intakeCloseCommand;
    public static IntakeCommand intakeWallCommand;
    public static IntakeCommand intakeFrontHighChamberCommand;
    public static IntakeCommand intakeLastLeftAutoCommand;
    public static IntakeCommand intakeRightFrontHighChamberCommand;
    public static IntakeCommand intakeAutoRightCommand;
    public static IntakeCommand intakeAutoRightGrabCommand;



    //commmand groups
    public static RetractAfterIntake retractAfterIntake;
    public static RetractFromBasket retractFromBasket;
    public static HighChamberCommand highChamberCommand;
    public static ScoreHighChamberCommand scoreHighChamberCommand;
    public static RetractAfterWallIntake retractAfterWallIntake;
    public static DropOffCommand dropOffCommandOp2;

    //test statics
    public static double x = 0, y = 0;
    public static double pitch = 0, roll = 0;

    //hardware
    public MotorEx BL, BR, FL, FR, arm, slideLeft, slideRight, slideNew;
    public MotorGroup slide;
    public ServoEx diffyLeft, diffyRight, claw;
    public Servo endStop;
    public AnalogInput armEncoder;
    public GoBildaPinpointDriver pinpoint;
    private MecanumDrive mecanumDrive;
    public IMU gyro;
    public RevColorSensorV3 sensor;

    //subsystems
    public DriveSubsystem driveSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public VisionSubsystem visionSubsystem;

    //system
    private LynxModule controlHub;

    //voltage
    private VoltageSensor voltageSensor;
    public static double batteryVoltage = 12;
    final double nominalVoltage = 12;
    public static double voltageCompensation = 1;
    private ElapsedTime voltageReadInterval = new ElapsedTime();

    //gamePads
    public GamepadEx m_driver;
    public GamepadEx m_driverOp;
    public Gamepad standardDriver1;
    public Gamepad standardDriver2;

    public CustomButton customButton;
    enum CustomButton {
        TOUCH, LEFTSTICKBUTTON, RIGHTSTICKBUTTON
    }
    public static Robot.CustomButton CustomButton;

    //random Todo: Need to clean up my loop time telemetry
    ElapsedTime time = new ElapsedTime();

    boolean manual = false;
    boolean flag = false;

    public void initialize(){
        time.reset();

        //general system
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        manual = false;

        //Pinpoint
        configurePinpoint();

        //dt
        FL = new MotorEx(hardwareMap, "FL");
        FR = new MotorEx(hardwareMap, "FR");
        BL = new MotorEx(hardwareMap, "BL");
        BR = new MotorEx(hardwareMap, "BR");
        FL.setRunMode(MotorEx.RunMode.RawPower);
        FR.setRunMode(MotorEx.RunMode.RawPower);
        BL.setRunMode(MotorEx.RunMode.RawPower);
        BR.setRunMode(MotorEx.RunMode.RawPower);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        FR.setInverted(true);
//        BR.setInverted(true);

        mecanumDrive = new MecanumDrive(FL, FR, BL, BR);
        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)
                )
        );

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL, mecanumDrive, telemetry, pinpoint);
        register(driveSubsystem);

        //arm
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_30);
        slideLeft = new MotorEx(hardwareMap, "slideL");
        slideRight = new MotorEx(hardwareMap, "slideR");
        slideNew = new MotorEx(hardwareMap, "slideNew");
        armEncoder = hardwareMap.get(AnalogInput.class, "armEncoder");
        endStop = hardwareMap.get(Servo.class, "backstop");
        arm.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setRunMode(Motor.RunMode.RawPower);
        slideRight.setRunMode(Motor.RunMode.RawPower);
        slideLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideLeft.setInverted(true);
        slideRight.setInverted(true);
        slideNew.setInverted(true);
        arm.setInverted(false);

        slide = new MotorGroup(slideLeft, slideRight, slideNew);

        armSubsystem = new ArmSubsystem(arm, slideRight, slide, endStop, armEncoder, telemetry);
        register(armSubsystem);

        //intake

        sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
        claw = new SimpleServo(hardwareMap, "claw", 0, 180, AngleUnit.DEGREES);
        diffyLeft =  new SimpleServo(hardwareMap, "diffyLeft", 0, 360, AngleUnit.DEGREES);
        diffyRight =  new SimpleServo(hardwareMap, "diffyRight", 0, 360, AngleUnit.DEGREES);

        intakeSubsystem = new IntakeSubsystem(claw, diffyLeft, diffyRight, telemetry);
        register(intakeSubsystem);

        //vision
        visionSubsystem = new VisionSubsystem(hardwareMap.get(WebcamName.class, "Webcam 1"), telemetry);
        register(visionSubsystem);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        configureCommands();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset());
        telemetry.addData("Y offset",pinpoint.getYOffset());
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();

        new ArmCoordinatesCommand(armSubsystem, armFoldX, armFoldY).schedule(true);
        
        CommandScheduler.getInstance().schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, 250));
        schedule(new InstantCommand(() -> armSubsystem.setEndstop(ArmSubsystem.Endstop.UP)));
    }

    @Override
    public void run(){
        if (!flag) {
            super.run(); //whatever you need to run once
            voltageReadInterval.reset();
            flag = true;
        }

        super.run();


        //voltage
        if(voltageReadInterval.seconds() >= 1){
            voltageReadInterval.reset();
            batteryVoltage = voltageSensor.getVoltage();
            voltageCompensation = batteryVoltage/nominalVoltage;
        }


        if (gamepad1.start){
            schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll));
        }


        //other telemetry
        telemetry.addData("manual", manualArm);
        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();
        //clear cache
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
        armFrontHighChamberCommand = new ArmCoordinatesCommand(armSubsystem, armFrontHighChamberX, armFrontHighChamberY);
        armHighBasketCommand = new ArmCoordinatesCommand(armSubsystem, armHighBasketX, armHighBasketY);
        //teleop high chamber
        armWhenHighChamberCommand = new ArmCoordinatesCommand(armSubsystem, armHighChamberX, armHighChamberY);

        //intaking
        //intake from sub
        //armWhenIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY);
        armInSubCommand = new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armInSubIntakeY);
        //intake from closer
        armWhenCloseIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY);
        //intaking from the wall
        armWhenIntakeWallCommand = new ArmCoordinatesCommand(armSubsystem, armIntakeWallX, armIntakeWallY);
        //arm auto parking
        armLeftAutoParkCommand = new ArmCoordinatesCommand(armSubsystem, armParkLeftAutoX, armParkLeftAutoY);
        // arm for auto right spcimen pick up
        armAutoRightCommand = new ArmCoordinatesCommand(armSubsystem, armAutoRightX, armAutoRightY);
        //drop command
        dropOffCommandOp2 = new DropOffCommand(armSubsystem, intakeSubsystem);




        //climbing
        armManualCommand = new ArmManualCommand(armSubsystem, m_driverOp::getRightY, m_driverOp::getLeftY);
        armPositionToClimb = new ArmCoordinatesCommand(armSubsystem, armPositionToClimbX, armPositionToClimbY);

        //INTAKE
        setIntakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll);


        //scoring
        intakeWhenHighBasketCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket);
        intakeWhenHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenHighChamber, rollWhenHighChamber);
        intakeFrontHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontHighChamber, rollFrontHighChamber);
        intakeRightFrontHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchFrontRightHighChamber, rollFrontRightHighChamber);
        //intakeRightScoreFrontHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchPlaceFrontHighRightChamber, rollPlaceFrontHighRightChamber);
        //intaking
        intakeLastLeftAutoCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchLastLeftAuto, rollLastLeftAuto);
        //intakeReadyCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);
        outakeReadyCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);

        intakeCloseCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);
        intakeWallCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchIntakeWall, rollIntakeWall);

        intakeCloseCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchWhenIntake, rollWhenIntake);

        // intake the left spike on right auto
        intakeAutoRightCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.EXTRAOPEN, pitchRightAutoSpecimen, rollRightAutoSpecimen);
        intakeAutoRightGrabCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchRightAutoSpecimen, rollRightAutoSpecimen);
        //home poses
        intakeWhenArmHomeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenArmHome);
        intakeWhenArmBackCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenBasket, rollWhenArmBack);
        intakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenIntake);

        //command groups
        retractAfterIntake = new RetractAfterIntake(armSubsystem, intakeSubsystem);
        retractFromBasket = new RetractFromBasket(driveSubsystem, armSubsystem, intakeSubsystem);
        highChamberCommand = new HighChamberCommand(armSubsystem, intakeSubsystem);
        scoreHighChamberCommand = new ScoreHighChamberCommand(armSubsystem, intakeSubsystem);
        retractAfterWallIntake = new RetractAfterWallIntake(armSubsystem, intakeSubsystem);

    }


    private void configurePinpoint() {
        telemetry.addLine("Configuring Pinpoint...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

       pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */

       pinpoint.setOffsets(0, -83.95); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
       pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
       pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


       //set yaw scalar

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
       pinpoint.recalibrateIMU();
       //pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset());
        telemetry.addData("Y offset",pinpoint.getYOffset());
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();
    }

}