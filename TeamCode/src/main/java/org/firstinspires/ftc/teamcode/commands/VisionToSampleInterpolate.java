package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.teamcode.other.Globals;
import org.firstinspires.ftc.teamcode.subSystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subSystems.VisionSubsystem;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class VisionToSampleInterpolate extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private BooleanSupplier slowMode;
    private DoubleSupplier strafe, forward, turn;


    public static double kPTurn = 0.005 * 180 / Math.PI;

    BasicPID turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

    private double prevSlidePosition = 0;

    private final double kPitchConversion = 2.33;

    private boolean offsetOnTarget = false;
    private boolean wristAngleOnTarget = false;

    private final double offsetTolerance = 7;

    private boolean hasFoundBlock = false;


    private InterpLUT lutXOffset = new InterpLUT(); //negative values report positive y poses
    private InterpLUT lutYOffset = new InterpLUT(); //

    private Pose2d samplePoseFieldOriented;

    double bruh = 0;

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.slowMode=slowMode;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        addRequirements(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem);

        lutXOffset.add(-99999999,-3.875);

        lutXOffset.add(-103.5,-3.875);
//        lutXOffset.add(-99.8,3.75);
        lutXOffset.add(-93,-3.5);
//        lutXOffset.add(-85.2,3.25);
        lutXOffset.add(-77.5,-3.0);
//        lutXOffset.add(-71.6,2.75);
        lutXOffset.add(-62.7,-2.5);
//        lutXOffset.add(-55.9,2.25);
        lutXOffset.add(-50.2,-2);
//        lutXOffset.add(-42.4,1.75);
        lutXOffset.add(-35.9,-1.5);
//        lutXOffset.add(-28,1.25);
        lutXOffset.add(-19.1,-1);
//        lutXOffset.add(-11.8,0.75);
        lutXOffset.add(-3.5,-0.5);
//        lutXOffset.add(-1,0.25);
        lutXOffset.add(0,0);

        lutXOffset.add(3.5,0.5);
        lutXOffset.add(19.1,1);
        lutXOffset.add(35.9,1.5);
        lutXOffset.add(50.2,2);
        lutXOffset.add(62.7,2.5);
        lutXOffset.add(77.5,3.0);
        lutXOffset.add(93,3.5);
        lutXOffset.add(103.5,3.875);
        lutXOffset.add(99999999,3.875);

        lutYOffset.add(-999999999,2.75);
        lutYOffset.add(-75.0,2.75);
        lutYOffset.add(-67.5,2.5);
        lutYOffset.add(-61,2.25);
        lutYOffset.add(-53.5,2.0);
        lutYOffset.add(-39.5,1.75);
        lutYOffset.add(-35.7,1.5);
        lutYOffset.add(-31.5,1.25);
        lutYOffset.add(-25.8,1.0);
        lutYOffset.add(-20.3,0.75);
        lutYOffset.add(-12,0.5);
        lutYOffset.add(-8,0.25);
        lutYOffset.add(0,0);
        lutYOffset.add(12.7,-0.5);
        lutYOffset.add(26.7,-1.0);
        lutYOffset.add(47.2,-1.5);
        lutYOffset.add(62.3,-2.0);
        lutYOffset.add(77.5,-2.5);
        lutYOffset.add(91,-3.0);
        lutYOffset.add(93.5,-3.5);
        lutYOffset.add(99999999,3.5);

        lutXOffset.createLUT();
        lutYOffset.createLUT();
    }

    @Override
    public void initialize(){
        intakeSubsystem.setDiffy(0,0);
        armSubsystem.setArm(5);
        driveSubsystem.setStartingPos(new Pose2d());
        hasFoundBlock=false;
    }

    @Override
    public void execute(){
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

        driveSubsystem.readPinpoint();
        Optional<List<Double>> allianceOffsets = visionSubsystem.getAllianceOffsets();

        if(allianceOffsets.isPresent()&&!hasFoundBlock){
            hasFoundBlock=true;
            Log.i("huhh", "hguhh");

            double xOffsetInches = lutXOffset.get(allianceOffsets.get().get(0));
            double yOffsetInches = lutYOffset.get(allianceOffsets.get().get(1));
            Transform2d cameraToSampleTransform = new Transform2d(new Translation2d(xOffsetInches,yOffsetInches), new Rotation2d()); //Y and X purposefully flipped
            Log.i("poseYOffset", String.valueOf(xOffsetInches));
            Log.i("poseXOffset", String.valueOf(yOffsetInches));
            Transform2d robotToCameraTransform = new Transform2d(new Translation2d(0,armSubsystem.getCurrentX()-5), new Rotation2d());
            samplePoseFieldOriented = driveSubsystem.getPos().plus(robotToCameraTransform).plus(cameraToSampleTransform);
            Log.i("poseSampleFieldX", String.valueOf(samplePoseFieldOriented.getTranslation().getX()));
            Log.i("poseSampleFieldY", String.valueOf(samplePoseFieldOriented.getTranslation().getY()));
            Log.i("poseRobotX", String.valueOf(driveSubsystem.getPos().getTranslation().getX()));
            Log.i("poseRobotY", String.valueOf(driveSubsystem.getPos().getTranslation().getY()));

        }

        if (hasFoundBlock){
            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();


            //CCW is positive
            double headingErrorRadians  = Math.atan2(-botToSample.getX(), botToSample.getY());
            double slideExtension = botToSample.getNorm();

            bruh++;
            if(bruh%100==10) {
                Log.i("boseRelativeX", String.valueOf(botToSample.getX()));
                Log.i("boseRelativeY", String.valueOf(botToSample.getY()));
                Log.i("boseRobotX", String.valueOf(driveSubsystem.getPos().getTranslation().getX()));
                Log.i("boseRobotY", String.valueOf(driveSubsystem.getPos().getTranslation().getY()));
                Log.i("boseSampleX", String.valueOf(samplePoseFieldOriented.getX()));
                Log.i("boseSampleY", String.valueOf(samplePoseFieldOriented.getY()));
                Log.i("stupidSlide", String.valueOf(slideExtension));
                Log.i("stupidturning", String.valueOf(180 * headingErrorRadians / (Math.PI)));
            }

//            Log.i("stupidX", String.valueOf(botToSample.getX()));
//            Log.i("stupidY", String.valueOf(botToSample.getY()));


//            double omega = MathUtils.clamp(turnpid.calculate(0, headingErrorRadians),-0.3,0.3);
//            Log.i("stupidOmega", String.valueOf(omega));
            double headingCalculation = turnpid.calculate(0, headingErrorRadians);
            double turnVelocity = Math.sqrt(Math.abs(headingCalculation)) * Math.signum(headingCalculation);
//            Log.i("stupidOmega", String.valueOf(turnVelocity));
            driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turnVelocity);
//
            armSubsystem.setSlide(slideExtension);

            Optional<Double> angleToSet = visionSubsystem.getAllianceSkew();
            if (angleToSet.isPresent()){
            intakeSubsystem.setDiffy(angleToSet.get()*kPitchConversion);
                wristAngleOnTarget=true;
            }
            else{
                wristAngleOnTarget=false;
            }
        }
    }

    @Override
    public void end(boolean e){
        intakeSubsystem.setDiffy(Globals.rollWhenIntake);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
