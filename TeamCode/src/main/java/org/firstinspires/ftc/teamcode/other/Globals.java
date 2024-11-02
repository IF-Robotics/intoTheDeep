package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int rollWhenReadyIntake = 150;
    public static int rollWhenIntake = 61;
    public static int rollWhenCloseIntake = 65;
    //intake power
    public static double intakePower = .7;
    public static double intakeHoldPower = .25;
    public static double outtakePower = -1;

    //scoring
    public static int pitchWhenBasket = 240;
    public static double outtakeBasketPower = -.25;
    public static double pitchWhenHighChamber = -100;
    public static double rollWhenHighChamber = -20;

    //home
    public static int rollWhenArmBack = -150;
    public static int rollWhenArmHome = 300;

//arm subsystem
    //arm home
    public static double armHomeX = 7;
    public static double armHomeY = 5;
    //arm back
    public static double armBackX = -3;
    public static double armBackY = 15;
    //arm high basket
    public static double armHighBasketX = -3;
    public static double  armHighBasketY = 43;
    //arm when intaking from sub
    public static double armIntakeX = 22;
    public static double armIntakeY = 4;
    //arm when close intake
    public static double armCloseIntakeX = 9.66;
    public static double armCloseIntakeY = 3.5;
    //arm when high chamber
    public static double armHighChamberX = -8;
    public static double armHighChamberY = 20.5;
    //manual arm boolean
    public static boolean manualArm = false;

//drive to point
    public static double translationKP = 0.0;
    public static double translationKI = 0.0;
    public static double translationKD = 0.0;
    public static double headingKP = 0.0;
    public static double headingKI = 0.0;
    public static double headingKD = 0.0;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0.0;

}
