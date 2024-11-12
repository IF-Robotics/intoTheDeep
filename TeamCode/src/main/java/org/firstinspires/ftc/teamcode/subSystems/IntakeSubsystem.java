package org.firstinspires.ftc.teamcode.subSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import static org.firstinspires.ftc.teamcode.other.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private ServoEx intake;
    private ServoEx diffyLeft, diffyRight;

    private Telemetry telemetry;

    private double pitchAngle;
    private double rollAngle;

    public static int pitchAngleOffset = 310;
    public static int rollAngleOffset = 60;

    //the value in the parentheses is our desired angle range in degrees
    public static double diffyScalar = 360/(350) * 355/255/*axon programming software is scaled to 255 degress max*/;

    //intakeing poses
    //first pos is normal, second is 45, third is 90, fourth is 135 degrees
    public int[] intakePitchPoses = {-90, 50, 200, 300};
    //roll when intaking should be -200

    public IntakeSubsystem(ServoEx intake, ServoEx diffyLeft, ServoEx diffyRight, Telemetry telemetry) {
        this.intake = intake;
        this.diffyLeft = diffyLeft;
        this.diffyRight = diffyRight;
        this.telemetry = telemetry;
    }

    public void openClaw (){
        intake.setPosition(clawOpen);
    }

    public void closeClaw (){
        intake.setPosition(clawClose);
    }

    public void setDiffy(double pitchAngle, double rollAngle){
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        //accounting for the fact that we are using 2:1 bevel gears
        pitchAngle/= 2;

        pitchAngle += pitchAngleOffset;
        rollAngle += rollAngleOffset;
        diffyLeft.turnToAngle(((pitchAngle + rollAngle) / 2) * diffyScalar);
        diffyRight.turnToAngle(((pitchAngle - rollAngle) / 2) * diffyScalar);
    }

    @Override
    public void periodic() {
        telemetry.addData("pitchAngle", pitchAngle);
        telemetry.addData("rollAngle", rollAngle);
    }

}
