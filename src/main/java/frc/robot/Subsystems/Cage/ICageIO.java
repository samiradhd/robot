package frc.robot.Subsystems.Cage;

import org.littletonrobotics.junction.AutoLog;

public interface ICageIO {
    @AutoLog
    public class CageStates {
   
        double Temp;
        double OutputCurrent;

     

        double cagePosition;
        double cageSpeed;
    }

    public CageStatesAutoLogged updateStates(CageStatesAutoLogged _CageStates);
}