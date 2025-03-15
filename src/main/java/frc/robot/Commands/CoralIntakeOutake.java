package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralEndEffector;

public class CoralIntakeOutake extends Command{

    CoralEndEffector effector;
    public boolean hasCoral;

    public CoralIntakeOutake(CoralEndEffector _effector){
        this.effector = _effector;
        addRequirements(this.effector);

    }

    @Override
    public void initialize(){
        if(this.effector.getTOFDistance() <= Constants.Coral.outTakeTreshhold){
            hasCoral = true;
        }
        else{
            hasCoral = false;
        }
    }

    @Override
    public void execute() {
        if(!hasCoral){
            effector.setEffectorPercentOutput(-0.3);
        }
        else{
            effector.setEffectorPercentOutput(-0.7);
        }
    }

    @Override
    public boolean isFinished(){
        return this.effector.getTOFDistance() < Constants.Coral.outTakeTreshhold && !hasCoral || this.effector.getTOFDistance() > Constants.Coral.outTakeTreshhold && hasCoral;

    }

    @Override
    public void end(boolean interupted){
        this.effector.setEffectorPercentOutput(0);

    }
    
}
