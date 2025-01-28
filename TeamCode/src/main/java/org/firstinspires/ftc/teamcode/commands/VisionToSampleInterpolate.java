package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.armSubIntakeY;
import static org.firstinspires.ftc.teamcode.other.Globals.rollWhenIntake;

import android.provider.Settings;
import android.util.Log;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.canvas.Rotation;
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
import org.opencv.core.RotatedRect;

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

    boolean isAuto;

    double autoDesiredHeading;

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, Boolean isAuto, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.slowMode=slowMode;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        this.isAuto = isAuto;

        addRequirements(visionSubsystem, armSubsystem, intakeSubsystem);

        if(!isAuto){
            addRequirements(driveSubsystem);
        }

        initializeLUTs();
    }

    public VisionToSampleInterpolate(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, Boolean isAuto){
        this(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem, isAuto, ()->false, ()->0, ()->0, ()->0);
    }

    private void initializeLUTs(){
        lutXOffset.add(-9999999, -8);
        lutXOffset.add(-130,-7.75);
        lutXOffset.add(-125,-7.5);
        lutXOffset.add(-116,-7.0);
        lutXOffset.add(-107,-6.5);
        lutXOffset.add(-99.6,-6.0);
        lutXOffset.add(-91,-5.5);
        lutXOffset.add(-81,-5.0);
        lutXOffset.add(-73,-4.5);
        lutXOffset.add(-64,-4.0);
        lutXOffset.add(-56.5,-3.5);
        lutXOffset.add(-47,-3.0);
        lutXOffset.add(-39.5,-2.5);
        lutXOffset.add(-31.5,-2.0);
        lutXOffset.add(-22.5,-1.5);
        lutXOffset.add(-14.1,-1.0);
        lutXOffset.add(-7.1,-0.5);
        lutXOffset.add(0,0);
        lutXOffset.add(7.1,0.5);
        lutXOffset.add(14.1,1.0);
        lutXOffset.add(22.5,1.5);
        lutXOffset.add(31.5,2.0);
        lutXOffset.add(39.5,2.5);
        lutXOffset.add(47,3.0);
        lutXOffset.add(56.5,3.5);
        lutXOffset.add(64,4.0);
        lutXOffset.add(73,4.5);
        lutXOffset.add(81,5.0);
        lutXOffset.add(91,5.5);
        lutXOffset.add(99.6,6.0);
        lutXOffset.add(107,6.5);
        lutXOffset.add(116,7.0);
        lutXOffset.add(125,7.5);
        lutXOffset.add(130,7.75);
        lutXOffset.add(9999999, 8);

        lutYOffset.add(-99999, 9.0);
        lutYOffset.add(-103,9.0);
        lutYOffset.add(-98.5,8.5);
        lutYOffset.add(-94,8.0);
        lutYOffset.add(-89,7.5);
        lutYOffset.add(-85.5,7.0);
        lutYOffset.add(-80.5,6.5);
        lutYOffset.add(-74.5,6.0);
        lutYOffset.add(-69.5,5.5);
        lutYOffset.add(-64.5,5.0);
        lutYOffset.add(-59,4.5);
        lutYOffset.add(-52,4.0);
        lutYOffset.add(-47.5,3.5);
        lutYOffset.add(-40.5,3);
        lutYOffset.add(-35.5,2.5);
        lutYOffset.add (-28,2.0);
        lutYOffset.add(-21.5,1.5);
        lutYOffset.add(-14,1.0);
        lutYOffset.add(-7.5,0.5);
        lutYOffset.add(0,0);
        lutYOffset.add(8.5,-0.5);
        lutYOffset.add(17.5,-1.0);
        lutYOffset.add(24.5,-1.5);
        lutYOffset.add(32,-2.0);
        lutYOffset.add(41.5,-2.5);
        lutYOffset.add(51,-3.0);
        lutYOffset.add(60,-3.5);
        lutYOffset.add(70,-4.0);
        lutYOffset.add(79,-4.5);
        lutYOffset.add(89.5,-5.0);
        lutYOffset.add(101,-5.5);
        lutYOffset.add(99999,-5.5);

        lutXOffset.createLUT();
        lutYOffset.createLUT();
    }

    @Override
    public void initialize(){
        intakeSubsystem.setDiffy(0,0);
        armSubsystem.setArmY(armReadySubIntakeY);
        hasFoundBlock=false;

        armSubsystem.setSlideP(0.1);
    }

    @Override
    public void execute(){
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

        driveSubsystem.readPinpoint();
        Optional<RotatedRect> allianceBoxFit = visionSubsystem.getAllianceBoxFit();

        if(allianceBoxFit.isPresent()&&!hasFoundBlock){
            hasFoundBlock=true;
            Log.i("huhh", "hguhh");

            List<Double> allianceOffsets = visionSubsystem.getOffsetFromBoxFit(allianceBoxFit.get());
            double xOffsetInches = 0.85*lutXOffset.get(allianceOffsets.get(0));
            double yOffsetInches = 0.85*lutYOffset.get(allianceOffsets.get(1));

            double allianceSkew = -visionSubsystem.getAngleFromRotatedRect(allianceBoxFit.get());

            Rotation2d allianceSkewRotation2d = new Rotation2d(Math.toRadians(allianceSkew));

            Transform2d cameraToSampleTransform = new Transform2d(new Translation2d(xOffsetInches,yOffsetInches), allianceSkewRotation2d);
            Transform2d robotToCameraTransform = new Transform2d(new Translation2d(0,armSubsystem.getCurrentX()), new Rotation2d());


            Log.i("poseYOffset", String.valueOf(xOffsetInches));
            Log.i("poseXOffset", String.valueOf(yOffsetInches));

            samplePoseFieldOriented = driveSubsystem.getPos().plus(robotToCameraTransform).plus(cameraToSampleTransform);
            Log.i("poseSampleFieldX", String.valueOf(samplePoseFieldOriented.getTranslation().getX()));
            Log.i("poseSampleFieldY", String.valueOf(samplePoseFieldOriented.getTranslation().getY()));
            Log.i("poseRobotX", String.valueOf(driveSubsystem.getPos().getTranslation().getX()));
            Log.i("poseRobotY", String.valueOf(driveSubsystem.getPos().getTranslation().getY()));

            intakeSubsystem.setDiffy(allianceSkew, rollWhenIntake);

            Translation2d botToSample = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getTranslation();

            autoDesiredHeading = Math.atan2(-botToSample.getX(), botToSample.getY());
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
            if(!isAuto){
                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turnVelocity);
            }
            else{
                driveSubsystem.pidToRotation2d(new Rotation2d(autoDesiredHeading));
            }
            armSubsystem.setArmX(slideExtension);

            double wristAngle = samplePoseFieldOriented.relativeTo(driveSubsystem.getPos()).getRotation().getDegrees();


            //Technically could remove the ifs put i think its makes it more understandable
            if(wristAngle>=105){
                while(wristAngle>=105){
                    wristAngle-=180;
                }
            }
            else if (wristAngle<=-105){
                while(wristAngle<=-105){
                    wristAngle+=180;
                }
            }

            intakeSubsystem.setDiffy(-wristAngle*kPitchConversion);
        }
        else{
            if(!isAuto) {
                driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
            }
        }
    }

    @Override
    public void end(boolean e){
        armSubsystem.setSlideP(0.2);
    }

    @Override
    public boolean isFinished(){
//        return false;
        boolean driveOnTarget=false;
        if(isAuto){
            if(hasFoundBlock && Math.abs(driveSubsystem.getPos().getRotation().getRadians()-autoDesiredHeading)<5){
                driveOnTarget=true;
            }
        }

        boolean slidesOnTarget = hasFoundBlock && Math.abs(armSubsystem.getTargetX()-armSubsystem.getSlideX())<2 && Math.abs(armSubsystem.getSlideVelocity())<0.5;

        return driveOnTarget && slidesOnTarget;
    }
}
