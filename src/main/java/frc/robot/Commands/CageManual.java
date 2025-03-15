package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Cage.Cage;

public class CageManual extends Command{
    Cage cage;
    Joystick joy;
    double position;

    public CageManual(Cage _cage, Joystick _joy){
        cage = _cage;
        joy = _joy;

    }  

    @Override
    public void execute() {
        if(joy.getPOV() == 180 && cage.getCageAbsPosition() > -36){
            cage.setCagePercentOutput(0.5);
            position = cage.getCagePosition();
        }

        else if(joy.getPOV() == 0 && cage.getCageAbsPosition() < 0){
            cage.setCagePercentOutput(-0.5);
            position = cage.getCagePosition();
        }

        else if(joy.getPOV() == 0 && cage.getCageAbsPosition() > 0){
            cage.setCagePosition(position);
        }

        else if(joy.getPOV() == 180 && cage.getCageAbsPosition() < -36){
            cage.setCagePosition(position);
        }

        else{
            cage.setCagePosition(position);
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(cage);
    }

}