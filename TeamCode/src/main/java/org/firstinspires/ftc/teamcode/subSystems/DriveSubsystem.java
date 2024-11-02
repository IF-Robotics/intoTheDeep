package org.firstinspires.ftc.teamcode.subSystems;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

import static org.firstinspires.ftc.teamcode.other.Globals.*;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import static org.firstinspires.ftc.teamcode.subSystems.OdoSubSystem.pos;

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
    private PIDController driveController;
    private PIDController headController;
    private double errorVectorX;
    private double errorVectorY;
    private double magnitude;
    private double theta;

    //constructor
    public DriveSubsystem(MotorEx FR, MotorEx FL, MotorEx BR, MotorEx BL) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
    }

    public void teleDrive(GamepadEx driver, boolean arcTanZones, int arcTanAngleRange, double strafeSpeed, double forwardSpeed, double turnSpeed) {
        y = forwardSpeed; // Remember, Y stick value is reversed
        x = strafeSpeed * 1.1; // Counteract imperfect strafing
        rx = -turnSpeed;

        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            power = .3;
        } else {
            power = 1;
        }

        if (arcTanZones) {
            if (Math.toDegrees(Math.atan(y / x)) > 90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 90 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) < -90 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) > -90 + arcTanAngleRange / 2) {
                x = 0;

                } else if (Math.toDegrees(Math.atan(y / x)) > 0 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 0 + arcTanAngleRange / 2
            || Math.toDegrees(Math.atan(y / x)) > 180 - arcTanAngleRange / 2 && Math.toDegrees(Math.atan(y / x)) < 180 + arcTanAngleRange / 2) {
                y = 0;
            }
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            FL.set(frontLeftPower * power);
            BL.set(backLeftPower * power);
            FR.set(frontRightPower * power);
            BR.set(backRightPower * power);

    }
    public void driveToPoint(double targetx, double targety, double targetheading){
        // Creates a PIDFController with gains kP, kI, kD, and kF


        driveController = new PIDController(kP, kI, kD);
        headController = new PIDController(headingkP, headingkI, headingkD);
        errorVectorX = targetx - pos.x;
        errorVectorY = targety - pos.y;
        magnitude = Math.hypot(errorVectorX, errorVectorY);

        theta = Math.atan2(errorVectorY, errorVectorX);


        //Kinematics

        power = 1;
        x = Math.cos(theta) * Math.sqrt(driveController.calculate(magnitude));
        y = Math.sin(theta) * Math.sqrt(headController.calculate(targetheading - pos.h));
        rx = Math.sqrt(headController.calculate(targetheading - pos.h));
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
        FL.set(frontLeftPower * power);
        BL.set(backLeftPower * power);
        FR.set(frontRightPower * power);
        BR.set(backRightPower * power);

    }

}
