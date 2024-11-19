package org.firstinspires.ftc.teamcode.subSystems;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.runner.Drawing;

public class DriveSubsystem extends SubsystemBase {

    private MotorEx FR, FL, BR, BL;

    private double y;
    private double x;
    private double rx;
    private double denominator;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;
    private double power = 1;

    private Telemetry telemetry;

    //driveToPoint squid
    private PIDController translationController, headingController;
    private double errorX, errorY, errorHeading;
    private GoBildaPinpointDriver pinpoint;
    private Pose2d currentPos;
    private double rawVectorMagnitude;
    private double correctedVectorMagnitude;
    private double vectorTheta;
    private double headingCalculation;

    private double strafeVelocity;
    private double forwardVelocity;
    private double turnVelocity;

    private MecanumDrive mecanumDrive;

    public Drive drive;
    public enum Drive {
        FAST,
        SLOW
    }
    ArmSubsystem armSubsystem;


    //constructor for auto
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, MecanumDrive mecanumDrive, Telemetry telemetry, GoBildaPinpointDriver pinpoint, ArmSubsystem armSubsystem) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        this.pinpoint = pinpoint;
        this.armSubsystem = armSubsystem;
    }

    //constructor for teleop
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, Telemetry telemtry) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.telemetry = telemtry;
    }

    public void stopDrive(){
        mecanumDrive.stop();
    }

    //current arm command
    //public Command currentArmCommand = armSubsystem.getCurrentCommand();

    //drive with arc tan dead zones (teleop)
    public void teleDrive(GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, double strafeSpeed, double forwardSpeed, double turnSpeed) {
        //slow mode
        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            power = .3;
        } else {
            power = 1;
        }


        //arc tan dead zones
        if (arcTanZones) {
            if (Math.toDegrees(Math.atan(y / x)) > 90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 90 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) < -90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) > -90 + arcTanAngleRange / 2) {
                x = 0;

                } else if (Math.toDegrees(Math.atan(y / x)) > 0 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 0 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) > 180 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 180 + arcTanAngleRange / 2) {
                y = 0;
            }
            }

            //actually moving
            mecanumDrive.driveRobotCentric(strafeSpeed * power, forwardSpeed * power, -turnSpeed * power);
    }

    public void driveToPoint(Pose2d targetPos){
        //completely temperary testing - delete if causing problems
        /*targetPos = new Pose2d(testX, testY, Rotation2d.fromDegrees(testHeading));*/

        //pids
        translationController = new PIDController(translationKP, translationKI, translationKD);
        headingController = new PIDController(headingKP, headingKI, headingKD);

        //error calculation
        errorX = currentPos.getX() - targetPos.getX();
        errorY = currentPos.getY() - targetPos.getY();
        errorHeading = currentPos.getRotation().getDegrees() - targetPos.getRotation().getDegrees();

        //testing
        telemetry.addData("errorX", errorX);
        telemetry.addData("errorY", errorY);
        telemetry.addData("errorHeading", errorHeading);

        //vector calculation
        rawVectorMagnitude = Math.hypot(errorX, errorY);
        vectorTheta = Math.toDegrees(Math.atan2(errorY, errorX));

        //pid calculation
        correctedVectorMagnitude = -Math.sqrt(Math.abs(translationController.calculate(0, rawVectorMagnitude))) * Math.signum(rawVectorMagnitude);
        headingCalculation = -headingController.calculate(targetPos.getRotation().getDegrees(), currentPos.getRotation().getDegrees());

        //testing
        telemetry.addData("rawVectorMagnitude", rawVectorMagnitude);
        telemetry.addData("correctedVectorMagnitude", correctedVectorMagnitude);
        telemetry.addData("vectorTheta", vectorTheta);

        //breaking vector into speed values + pid
        strafeVelocity = lateralMutliplier * (Math.cos (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        forwardVelocity = (Math.sin (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        turnVelocity = Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);

        //testing
        telemetry.addData("strafeSpeed", strafeVelocity);
        telemetry.addData("forwardSpeed", forwardVelocity);
        telemetry.addData("turnSpeed", turnVelocity);

        //actually driving
        mecanumDrive.driveFieldCentric(strafeVelocity, forwardVelocity, turnVelocity, getHeadingInDegrees(currentPos));
    }

    public void readPinpoint() {
        pinpoint.update();
        Pose2D tempPos = pinpoint.getPosition();
        currentPos = new Pose2d(-tempPos.getY(DistanceUnit.INCH), tempPos.getX(DistanceUnit.INCH), Rotation2d.fromDegrees(tempPos.getHeading(AngleUnit.DEGREES)));
        telemetry.addData("xDTPos", currentPos.getX());
        telemetry.addData("yDTPos", currentPos.getY());
        telemetry.addData("dtHeading", currentPos.getRotation().getDegrees());

        drawBot(currentPos);
    }

    public Pose2d getPos(){
        return currentPos;
    }

    public double getTranslationalError(){
        return rawVectorMagnitude;
    }

    public double getHeadingError(){
        return errorHeading;
    }

    public void setStartingPos(Pose2d pos){
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH, pos.getY(), -pos.getX(), AngleUnit.RADIANS, pos.getHeading()));
    }

    public static double getHeadingInDegrees(Pose2d pose) {
            // Convert the heading from radians to degrees
            double headingInDegrees = pose.getRotation().getDegrees();

            // Normalize the heading to 0-360 degrees
            if (headingInDegrees < 0) {
                headingInDegrees += 360;
            }

            return headingInDegrees;
    }

    public void drawBot(Pose2d pose) {
        //drawing robot on dash
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), new com.acmerobotics.roadrunner.Pose2d(pose.getX(), pose.getY(), pose.getRotation().getRadians() + Math.PI/2));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}