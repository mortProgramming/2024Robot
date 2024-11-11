package org.mort11.commands.actions.endeffector.pos;

import org.mort11.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class WristToPos extends Command {
    private Wrist wrist;

    private double target;

    public WristToPos(double setpoint){
        this.target = setpoint;

        wrist = Wrist.getInstance();
        
        addRequirements(wrist);
    }
    
    @Override
    public void execute() {
        wrist.setSetpoint(target);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
