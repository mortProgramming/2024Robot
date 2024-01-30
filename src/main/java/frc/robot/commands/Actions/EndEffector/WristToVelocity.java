package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristToVelocity extends Command{
    private Wrist wrist;
    private double velocity;

    public WristToVelocity(double velocity){
        this.velocity = velocity;
        wrist = wrist.getInstance();
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        wrist.setWristVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
