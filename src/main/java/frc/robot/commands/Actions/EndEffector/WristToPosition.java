package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristToPosition extends Command {
    private Wrist wrist;

    private double target;

    public WristToPosition(double setpoint){
        this.target = setpoint;
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }
    
    @Override
    public void initialize() {
        wrist.setSetPoint(target);
    }

    @Override
    public void execute() {
        wrist.setPIDorVelocity(false);
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setPIDorVelocity(true);;
    }

    @Override
    public boolean isFinished() {
        return wrist.nearSetpoint();
    }
}
