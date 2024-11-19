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

    public static int pitchAngleOffset = 295;
    public static int rollAngleOffset = 60;

    //the value in the parentheses is our desired angle range in degrees
    public static double diffyScalar = 360/(350) * 355/255/*axon programming software is scaled to 255 degress max*/;

    //intake rotation
    private int intakePitchAngle = 0;

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

    /*public void clawExtraOpen () {
        intake.setPosition(clawExtraOpen);
    }*/

    public void setDiffy(double pitchAngle, double rollAngle){
        this.pitchAngle = pitchAngle;
        this.rollAngle = rollAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        //accounting for the fact that we are using 2:1 bevel gears
        pitchAngle/= 2;

        diffyLeft.turnToAngle((((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
        diffyRight.turnToAngle((((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
    }

    public void setDiffy(double pitchAngle){
        this.pitchAngle = pitchAngle;

        //pitch is like the wrist and roll is like the twist             if that makes any sense at all

        //accounting for the fact that we are using 2:1 bevel gears
        pitchAngle/= 2;

        diffyLeft.turnToAngle((((pitchAngle + pitchAngleOffset) + (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
        diffyRight.turnToAngle((((pitchAngle + pitchAngleOffset) - (rollAngle + rollAngleOffset)) / 2) * diffyScalar);
    }


    public void resetRotateIntake(){
        intakePitchAngle = 0;
    }

    public void rotateIntake(){
        //switching to next rotation
        intakePitchAngle += 45;

        //rotation reset
        if (intakePitchAngle > 135){
            intakePitchAngle = 0;
        }
        switch (intakePitchAngle){
            case 0:
                setDiffy(pitchesWhenIntake[0]);
                break;
            case 45:
                setDiffy(pitchesWhenIntake[1]);
                break;
            case 90:
                setDiffy(pitchesWhenIntake[2]);
                break;
            case 135:
                setDiffy(pitchesWhenIntake[3]);
                break;
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("pitchAngle", pitchAngle);
        telemetry.addData("rollAngle", rollAngle);
        telemetry.addData("clawPos", intake.getPosition());
    }

}
