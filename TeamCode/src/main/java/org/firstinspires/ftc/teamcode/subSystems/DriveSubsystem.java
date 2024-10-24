package org.firstinspires.ftc.teamcode.subSystems;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase{

    private DcMotorEx FR, FL, BR, BL;

    private double y;
    private double x;
    private double rx;
    private double denominator;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

    //constructor
    public DriveSubsystem(DcMotorEx FR, DcMotorEx FL, DcMotorEx BR, DcMotorEx BL) {
        this.FR = FR;
        this.FL = FL;
        this.BR = BR;
        this.BL = BL;
    }

    public void teleDrive(double power, double strafeSpeed, double forwardSpeed, double turnSpeed){
        y = forwardSpeed; // Remember, Y stick value is reversed
        x = strafeSpeed * 1.1; // Counteract imperfect strafing
        rx = turnSpeed;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        FL.setPower(frontLeftPower * power);
        BL.setPower(backLeftPower * power);
        FR.setPower(frontRightPower * power);
        BR.setPower(backRightPower * power);
    }

}
