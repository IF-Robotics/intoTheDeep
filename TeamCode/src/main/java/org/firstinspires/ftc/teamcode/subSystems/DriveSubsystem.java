package org.firstinspires.ftc.teamcode.subSystems;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

import static org.firstinspires.ftc.teamcode.other.Globals.*;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private double power;

    private Telemetry telemetry;

    //driveToPoint squid
    private PIDController translationController, headingController;
    private double errorX, errorY, errorHeading;
    private SparkFunOTOS otos;
    private Pose2d currentPos;
    private double rawVectorMagnitude;
    private double correctedVectorMagnitude;
    private double vectorTheta;
    private double headingCalculation;

    private double strafeVelocity;
    private double forwardVelocity;
    private double turnVelocity;

    private MecanumDrive fieldCentricDrive;


    //constructor for auto
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, MecanumDrive fieldCentricDrive, Telemetry telemetry, SparkFunOTOS otos) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.fieldCentricDrive = fieldCentricDrive;
        this.telemetry = telemetry;
        this.otos = otos;
    }

    //constructor for teleop
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, Telemetry telemtry) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
        this.telemetry = telemtry;
    }

    //basic drive
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){
        y = forwardSpeed; // Remember, Y stick value is reversed
        x = strafeSpeed * 1.1; // Counteract imperfect strafing
        rx = -turnSpeed;

        //mec inverse kinematics
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = ((y + x + rx) / denominator) * power;
        backLeftPower = ((y - x + rx) / denominator) * power;
        frontRightPower = ((y - x - rx) / denominator) * power;
        backRightPower = ((y + x - rx) / denominator) * power;

        //writes
        FL.set(frontLeftPower);
        BL.set(backLeftPower);
        FR.set(-frontRightPower);
        BR.set(-backRightPower);

        telemetry.addData("dt power", frontLeftPower + backLeftPower + frontRightPower + backRightPower);
    }

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
            drive(Math.abs(strafeSpeed), Math.abs(forwardSpeed), Math.abs(turnSpeed));
    }

    public void driveToPoint(Pose2d targetPos){
        /*//completely temperary testing - delete if causing problems
        targetPos = new Pose2d(testX, testY, Rotation2d.fromDegrees(testHeading));*/

        //pids
        translationController = new PIDController(translationKP, translationKI, translationKD);
        headingController = new PIDController(headingKP, headingKI, headingKD);

        //error calculation
        errorX = currentPos.getX() - targetPos.getX();
        errorY = currentPos.getY() - targetPos.getY();
        errorHeading = targetPos.getHeading() - currentPos.getRotation().getDegrees();

        //testing
        telemetry.addData("errorX", errorX);
        telemetry.addData("errorY", errorY);
        telemetry.addData("errorHeading", errorHeading);

        //vector calculation
        rawVectorMagnitude = Math.hypot(errorX, errorY);
        vectorTheta = Math.toDegrees(Math.atan2(errorY, errorX));

        //pid calculation
        correctedVectorMagnitude = Math.sqrt(Math.abs(translationController.calculate(0, rawVectorMagnitude))) * Math.signum(rawVectorMagnitude);
        headingCalculation = headingController.calculate(targetPos.getRotation().getDegrees(), currentPos.getRotation().getDegrees());

        //testing
        telemetry.addData("rawVectorMagnitude", rawVectorMagnitude);
        telemetry.addData("correctedVectorMagnitude", correctedVectorMagnitude);
        telemetry.addData("vectorTheta", vectorTheta);

        //breaking vector into speed values + pid   jj
        strafeVelocity = - lateralMutliplier * (Math.cos (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        forwardVelocity = - (Math.sin (Math.toRadians(vectorTheta)) * correctedVectorMagnitude);
        turnVelocity = - Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);

        //testing
        telemetry.addData("strafeSpeed", strafeVelocity);
        telemetry.addData("forwardSpeed", forwardVelocity);
        telemetry.addData("turnSpeed", turnVelocity);

        //actually driving
        power = 1;
        fieldCentricDrive.driveFieldCentric(strafeVelocity, forwardVelocity, turnVelocity, currentPos.getRotation().getDegrees());
    }

    public void readOtos() {
        SparkFunOTOS.Pose2D sparkfunPos = otos.getPosition();
        currentPos = new Pose2d(sparkfunPos.x, sparkfunPos.y, new Rotation2d(Math.toRadians(sparkfunPos.h)));
        telemetry.addData("xDTPos", currentPos.getX());
        telemetry.addData("yDTPos", currentPos.getY());
        telemetry.addData("dtHeading", currentPos.getRotation().getDegrees());
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

    public void setStaringPos(SparkFunOTOS.Pose2D pos){
        otos.setPosition(pos);
    }

}