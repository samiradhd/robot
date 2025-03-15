package frc.robot.Subsystems.Cage;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Cage implements ICageIO, Subsystem{

    SparkMax cage;
    SparkMaxConfig cageconfig;
    SparkClosedLoopController cagePID;
    RelativeEncoder cageRelativeEncoder;
    DutyCycleEncoder absencoder;

    CageStatesAutoLogged cageStates = new CageStatesAutoLogged();


    public Cage(int cagerMotorCANID){
        cage = new SparkMax(cagerMotorCANID, MotorType.kBrushless);
        cageconfig = new SparkMaxConfig();
        cagePID = cage.getClosedLoopController();
        cageRelativeEncoder = cage.getEncoder();
        absencoder = new DutyCycleEncoder(Constants.Cage.absEncoderChannel);

        cageconfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.Cage.cageCurrentLimit);
        cageconfig.encoder
            .positionConversionFactor(Constants.Cage.rotToDeg)
            .velocityConversionFactor(Constants.Cage.rotToDeg / 60);
        cageconfig.closedLoop
            .outputRange(-0.7,0.7)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1,0,0);

        cage.configure(cageconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       

    }
    @Override
    public void periodic() {
        updateStates(cageStates);
        Logger.processInputs("cage", cageStates);
    }

    @Override
    public CageStatesAutoLogged updateStates(CageStatesAutoLogged _cageStates) {
        
        _cageStates.OutputCurrent = cage.getOutputCurrent();
        _cageStates.Temp = cage.getMotorTemperature();

        _cageStates.cagePosition= getCagePosition();
        _cageStates.cageSpeed = getCageSpeed();
        
        return _cageStates;
    }

    public double getCagePosition(){
        return cageRelativeEncoder.getPosition();
    }

    public double getCageAbsPosition(){
        return absencoder.get()*360-242;
    }

    public double getCageSpeed(){
        return cageRelativeEncoder.getVelocity();
    }

    public void setCagePercentOutput(double speed){
        cage.set(speed);
    }

    public void setCagePosition(double angle){
        cagePID.setReference(angle, ControlType.kPosition);
    }
   
}

    

