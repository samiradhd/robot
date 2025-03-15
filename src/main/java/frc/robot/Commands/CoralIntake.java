package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;

public class CoralIntake extends Command{

    CoralElevator elevator;
    CoralEndEffector effector;

    public CoralIntake(CoralElevator _elevator, CoralEndEffector _effector){
        this.elevator = _elevator;
        this.effector = _effector;
        addRequirements(this.elevator, this.effector);

    }

    @Override
    public void initialize(){
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
    }

    @Override
    public void execute() {
        this.effector.setEffectorPercentOutput(-0.3);
    }
    @Override
    public boolean isFinished(){
        return this.effector.getTOFDistance() < Constants.Coral.outTakeTreshhold;

    }

    @Override
    public void end(boolean interupted){
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
        this.effector.setEffectorPercentOutput(0);

    }

    
}
