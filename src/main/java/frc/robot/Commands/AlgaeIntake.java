package frc.robot.Commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Algae.Algae;

public class AlgaeIntake extends Command{

    Algae algae;
    private final Timer timer = new Timer();

    public AlgaeIntake(Algae _algae){
        this.algae = _algae;
        addRequirements(this.algae);
    }
    
    @Override
    public void initialize(){
        this.algae.setAnglerPosition(Constants.Algae.intakePos);
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        this.algae.setIntakePercentOutput(-1);
    }

    @Override
    public boolean isFinished() {
        return algae.getIntakeCurrent() > 10 && timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interupted) {
        this.algae.setAnglerPosition(0);
        this.algae.setIntakePercentOutput(0);
        timer.stop();
    }

}
