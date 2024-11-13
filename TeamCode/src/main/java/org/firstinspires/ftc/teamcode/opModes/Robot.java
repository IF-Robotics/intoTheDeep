package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCoordinatesCommand;
import org.firstinspires.ftc.teamcode.commands.ArmManualCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
@Config
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
    public ServoEx diffyLeft, diffyRight, claw;
    public AnalogInput armEncoder;
    public GoBildaPinpointDriver pinpoint;
    private MecanumDrive fieldCentricDrive;

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

        fieldCentricDrive = new MecanumDrive(FL, FR, BL, BR);

        driveSubsystem = new DriveSubsystem(FR, FL, BR, BL, fieldCentricDrive, telemetry, pinpoint);
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
        claw = new SimpleServo(hardwareMap, "claw", 0, 180, AngleUnit.DEGREES);
        diffyLeft =  new SimpleServo(hardwareMap, "diffyLeft", 0, 360, AngleUnit.DEGREES);
        diffyRight =  new SimpleServo(hardwareMap, "diffyRight", 0, 360, AngleUnit.DEGREES);

        intakeSubsystem = new IntakeSubsystem(claw, diffyLeft, diffyRight, telemetry);
        register(intakeSubsystem);

        m_driver = new GamepadEx(gamepad1);
        m_driverOp = new GamepadEx(gamepad2);

        configureCommands();

        //Pinpoint
        configurePinpoint();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset());
        telemetry.addData("Y offset",pinpoint.getYOffset());
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();
    }

    @Override
    public void run(){
        if (!flag) {
            super.run(); //whatever you need to run once
            flag = true;
        }

        super.run();

        //clear cache
        //other telemetry
        telemetry.addData("manual", manualArm);
        //loopTime
        telemetry.addData("hz ", 1/(time.seconds()));
        telemetry.update();
        time.reset();
        controlHub.clearBulkCache();

        if (gamepad1.start){
            schedule(new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll));
        }
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
        //intake from sub
        armWhenIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armReadySubIntakeX, armReadySubIntakeY);
        //intake from closer
        armWhenCloseIntakeCommand = new ArmCoordinatesCommand(armSubsystem, armCloseIntakeX, armCloseIntakeY);


        armManualCommand = new ArmManualCommand(armSubsystem, m_driverOp, m_driverOp::getRightY, m_driverOp::getLeftY);

        //INTAKE
        setIntakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitch, roll);

        //scoring
        intakeWhenHighBasketCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenBasket, rollWhenBasket  );
        intakeWhenHighChamberCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, pitchWhenHighChamber, rollWhenHighChamber);

        //intaking
        intakeReadyCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);
        outakeReadyCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);
        intakeCloseCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenIntake, rollWhenIntake);

        //home poses
        intakeWhenArmHomeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.CLOSE, 0, rollWhenArmHome);
        intakeWhenArmBackCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, pitchWhenBasket, rollWhenArmBack);
        intakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.Claw.OPEN, 0, rollWhenIntake);

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
        //pinpoint.setYawScalar(1.0);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
       pinpoint.recalibrateIMU();
       pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset",pinpoint.getXOffset());
        telemetry.addData("Y offset",pinpoint.getYOffset());
        telemetry.addData("pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Version Number:",pinpoint.getDeviceVersion());
        telemetry.addData("Device SCalar",pinpoint.getYawScalar());
        telemetry.update();
    }
}