package org.firstinspires.ftc.teamcode.other;

import com.acmerobotics.dashboard.config.Config;

import java.util.function.DoubleSupplier;

@Config
public class Globals {
//intake subsystem
    //intaking
    public static int pitchWhenIntake = 0;
    public static int[] pitchesWhenIntake = {0/*normal*/, 105, 210/*OrthogonalIntake*/, 315};
    public static int rollWhenIntake = -190;
    public static int pitchLastLeftAuto = 60;
    public static int rollLastLeftAuto = -190;
    //intake from the wall
    public static int pitchIntakeWall = 400;
    public static int rollIntakeWall = -20;
    // Right left specimens
    public static int pitchRightAutoSpecimen = pitchIntakeWall;
    public static int rollRightAutoSpecimen = -190;

    //claw poses
    public static double clawOpen = 0.7;
    public static double clawExtraOpen = .8;
    public static double clawClose = .25;


    //scoring
    //basket
    public static int pitchWhenBasket = pitchIntakeWall;
    public static int rollWhenBasket = 130;

    public static double pitchWhenHighChamber = 0;
    public static double rollWhenHighChamber = -20;

    //autoLeft
    public static double autoPitchFrontHighChamber = 0;
    public static double rollFrontHighChamber = 70+20;

    public static double pitchFrontHighChamber = 300;


    //teleop high chamber
    public static double pitchTeleopHighChamber = 0;
    public static double rollTeleopHighChamber = 230;


    //autoright
    public static double pitchFrontRightHighChamber = 0;
    public static double rollFrontRightHighChamber = rollTeleopHighChamber;
    public static double pitchPlaceFrontHighRightChamber = 0;
    public static double rollPlaceFrontHighRightChamber = 260;


    //home
    public static int rollWhenArmBack = -150;
    public static int rollWhenArmHome = 200;

//arm subsystem
    //arm home
    public static double armHomeX = 7.5;
    public static double armHomeY = 7;
    //arm fold
    public static double armFoldX = 6.5;
    public static double armFoldY = 4.0;
    //arm back
    public static double armBackX = -1;
    public static double armBackY = 17;
    //arm high basket
    public static double armHighBasketX = -2;
    public static double  armHighBasketY = 45;
    //arm when front high chamber
    public static double armFrontHighChamberX = 20;
    public static double armFrontHighChamberY = 17.6;
    public static double autoArmFrontHighChamberY = 17.75;
    //arm when high chamber
    public static double armRightHighChamberX = -1;
    public static double armRightHighChamberY = 28.0;
    public static double armHighChamberX = -1;
    public static double armHighChamberY = armRightHighChamberY;

    //arm when intaking from sub
    public static double armAutoSpikeX = 23;
    public static double armReadySubIntakeX = 27;
    public static double armSubIntakeY = 3;
    public static double armReadySubIntakeY = 8;
    public static double armInSubIntakeY = 5;
    //arm when close (distance) intake
    public static double armCloseIntakeX = 15;
    public static double armCloseIntakeY = 6;
    //arm when intaking form the wall
    public static double armIntakeWallX = 7.7;
    public static double armIntakeWallY = 8 +0.5;
    // arm when intaking for AutoRight
    public static double armAutoRightX = 16.6;
    public static double armAutoRightY = 1.3;
    public static double armAutoPushY = -1.5;
    public static double armAutoReadyPushY = 5;

    //manual arm boolean
    public static boolean manualArm = false;
    public static boolean manualSlides = false;
    //arm auto park
    public static double armParkLeftAutoX = 13;
    public static double armParkLeftAutoY = 23;

    //endstop
    public static double endstopUp = .463;
    public static double endstopDown = 1;

    //arm when climbing
    //climbing to first rung
    public static double armAngleReady = 36.08;
    public static double armPositionToClimbX = 16.9;
    public static double armPositionToClimbY = 24.8;
    //climbing to second rung
    public static double armAngleToSecondRungX = 3.5;
    public static double armAngleToSecondRungY = 14.6;
    public static double armExtendPastSecondRungX = 6;//12;
    public static double armExtendPastSecondRungY = 27.5;
    public static double armMoveToSecondRungX = 15;
    public static double armMoveToSecondRungY = 27;
    public static double armPositionRobotToEdgeOfFirstRungX = -8;
    public static double armPositionRobotToEdgeOfFirstRungY = 18;
    //completely retracting when climbing
    public static double armCompleteRetractX = 8;
    public static double armCompleteRetractY = 3.5;

//drive to point
    public static double translationKP = 0.02*1.2;

    public static double translationKI = 0.0;
    public static double translationKD = 0.01;
//    public static double translationKD = 0.001;

    public static double translationKR = .5; //KR is the constant for the root of the pid
    public static double translationMaxVel = 1; //in inches per second

    public static double headingKP = 0.005 * (11.5/9.0);
    public static double headingKI = 0.0;
    public static double headingKD = 0.0002;
    public static double headingKR = .5;
    public static double headingMaxVel = 1; //in degrees per second
    public static double lateralMutliplier = 1.5;

    public static double testX = 0.0;
    public static double testY = 0.0;
    public static double testHeading = 0;

    //teleop mode
    public static boolean teleopSpec = false;

}
