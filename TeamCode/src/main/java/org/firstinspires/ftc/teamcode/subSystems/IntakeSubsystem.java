package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private CRServo intake;
    private ServoEx diffyLeft, diffyRight;

    private Telemetry telemetry;

    public static int pitchAngleOffset = 330;
    public static int rollAngleOffset = -20;

    public IntakeSubsystem(CRServo intake, ServoEx diffyLeft, ServoEx diffyRight, Telemetry telemetry) {
        this.intake = intake;
        this.diffyLeft = diffyLeft;
        this.diffyRight = diffyRight;
        this.telemetry = telemetry;
    }

    public void setIntake(double power) {
        intake.set(power);
    }

    public void setDiffy(double pitchAngle, double rollAngle){
        pitchAngle += pitchAngleOffset;
        rollAngle += rollAngleOffset;
        diffyLeft.turnToAngle((pitchAngle + rollAngle) / 2);
        diffyRight.turnToAngle((pitchAngle - rollAngle) / 2);
    }

}
