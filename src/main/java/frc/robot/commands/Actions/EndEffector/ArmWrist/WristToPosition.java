package frc.robot.commands.Actions.EndEffector.ArmWrist;

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
        wrist.setVelocityMode(false);
        wrist.setSetPoint(target);
    }

    @Override
    public void execute() {
        // wrist.setVelocityMode(false);
    }

    @Override
    public void end(boolean interrupted) {
        // wrist.setVelocityMode(true);
    }

    @Override
    public boolean isFinished() {
        return wrist.nearSetpoint();
    }
}
