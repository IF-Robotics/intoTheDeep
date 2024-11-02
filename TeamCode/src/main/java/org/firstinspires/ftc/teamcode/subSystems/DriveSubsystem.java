package org.firstinspires.ftc.teamcode.subSystems;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    private SparkFunOTOS.Pose2D currentPos;
    private double rawVectorMagnitude;
    private double correctedVectorMagnitude;
    private double vectorTheta;

    private double strafeSpeed;
    private double forwardSpeed;
    private double turnSpeed;



    //constructor for auto
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL, Telemetry telemetry, SparkFunOTOS otos) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
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
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        //writes
        FL.set(frontLeftPower * power);
        BL.set(backLeftPower * power);
        FR.set(frontRightPower * power);
        BR.set(backRightPower * power);
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
            drive(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveToPoint(SparkFunOTOS.Pose2D targetPos){

        //error calculation
        errorX = targetPos.x - currentPos.x;
        errorY = targetPos.y - currentPos.y;
        errorHeading = targetPos.h - currentPos.h;

        //vector calculation
        rawVectorMagnitude = Math.hypot(errorX, errorY);
        vectorTheta = Math.atan2(errorY, errorX);

        //pid calculation
        correctedVectorMagnitude = translationController.calculate(rawVectorMagnitude);

        //breaking vector into speed values + pid calc
        strafeSpeed = Math.cos(vectorTheta) * Math.sqrt(correctedVectorMagnitude);
        forwardSpeed = Math.sin(vectorTheta) * Math.sqrt(correctedVectorMagnitude);
        turnSpeed = headingController.calculate(errorHeading);

        //actually driving
        drive(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void readOtos() {
        currentPos = otos.getPosition();
        telemetry.addData("xDTPos", currentPos.x);
        telemetry.addData("yDTPos", currentPos.y);
        telemetry.addData("dtHeading", currentPos.h);
    }

    public SparkFunOTOS.Pose2D getPos(){
        return currentPos;
    }

    public void setStaringPos(SparkFunOTOS.Pose2D pos){
        otos.setPosition(pos);
    }

}