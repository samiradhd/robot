package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Algae.AlgaeDescore;

public class DescoreAlgae extends Command{
    AlgaeDescore descore;
    Joystick joy;

    public DescoreAlgae(AlgaeDescore _descore, Joystick _joy){
        descore = _descore;
        joy = _joy;

    }  

    @Override
    public void execute() {
        if(joy.getPOV() == 90 ){
            descore.setDescorePercentOutput(0.4);
        }

        else if(joy.getPOV() == 270){
            descore.setDescorePercentOutput(-0.4);
        }

        else{
            descore.setDescorePercentOutput(0);
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(descore);
    }

}