package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeX;
import static org.firstinspires.ftc.teamcode.other.Globals.armReadySubIntakeY;

import androidx.core.math.MathUtils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class VisionToSample extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private BooleanSupplier slowMode;
    private DoubleSupplier strafe, forward, turn;

    public static double kPSlides = -0.005;

    BasicPID slidePid = new BasicPID(new PIDCoefficients(kPSlides,0,0));


    public static double kPTurn = 0.005;

    BasicPID turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));

    private double prevSlidePosition = 0;

    private final double kPitchConversion = 2.33;

    private boolean offsetOnTarget = false;
    private boolean wristAngleOnTarget = false;

    private final double offsetTolerance = 7;

    ElapsedTime timer = new ElapsedTime();


    public VisionToSample(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier slowMode, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.slowMode=slowMode;
        this.strafe=strafe;
        this.forward=forward;
        this.turn=turn;

        addRequirements(driveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize(){
//        prevSlidePosition = armSubsystem.getSlideExtention();
        Globals.manualSlides=true;
        intakeSubsystem.setDiffy(0,0);
        timer.reset();
    }

    @Override
    public void execute(){


        slidePid = new BasicPID(new PIDCoefficients(kPSlides,0,0));
        turnpid = new BasicPID(new PIDCoefficients(kPTurn,0,0));
        Optional<List<Double>> allianceOffsets = visionSubsystem.getAllianceOffsets();

        armSubsystem.setArm(4);
        if(allianceOffsets.isPresent()){
            //ADJUST MIN AND MAX AS NECCESSARY
//            double slideCompensation = slidePid.calculate(0,allianceOffsets.get().get(1));
//            prevSlidePosition = MathUtils.clamp(prevSlidePosition + slideCompensation,15,30);
//            armSubsystem.setSlide(prevSlidePosition);

            armSubsystem.setArmCoordinates(armReadySubIntakeX, armReadySubIntakeY);
            double pidOutput = slidePid.calculate(allianceOffsets.get().get(1),0);
            double clampedPidOutput=MathUtils.clamp(pidOutput,-0.3,0.3);
            armSubsystem.setSlidePower(pidOutput);
//            armSubsystem.setSlidePower(0);

            double omega = MathUtils.clamp(turnpid.calculate(allianceOffsets.get().get(0),0),-0.3,0.3);
            driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), omega);
            if( (Math.abs(allianceOffsets.get().get(0))<offsetTolerance) && (Math.abs(allianceOffsets.get().get(1))<offsetTolerance) ){
                offsetOnTarget = true;
            }
            else{
                offsetOnTarget = false;
                timer.reset();
            }
        }
        else {
            armSubsystem.setPowerZero();
            //Just asuuming the arctan arguements are constant, prob wont ever change
            driveSubsystem.teleDrive(slowMode, true, 10, strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
        }

        Optional<Double> angleToSet = visionSubsystem.getAllianceSkew();
        if (angleToSet.isPresent()){
            intakeSubsystem.setDiffy(angleToSet.get()*kPitchConversion);
            wristAngleOnTarget=true;
        }
        else{
            wristAngleOnTarget=false;
        }
    }

    @Override
    public void end(boolean e){
        Globals.manualSlides=false;
        double slideSetpoint = armSubsystem.getSlideExtention();
        slideSetpoint-=2.5;
        if (slideSetpoint<8){
            slideSetpoint=8;
        }
        armSubsystem.setSlide(slideSetpoint);
        intakeSubsystem.setDiffy(Globals.rollWhenIntake);
    }

    @Override
    public boolean isFinished(){
        if(offsetOnTarget&&wristAngleOnTarget){
            return true;
//            return false;
        }
        return false;
    }
}
