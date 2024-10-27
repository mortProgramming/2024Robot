package org.mort11.commands.actions.endeffector.armwrist;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Wrist;

public class WristToVelocity extends Command{
    private Wrist wrist;
    private DoubleSupplier velocity;

    public WristToVelocity(DoubleSupplier velocity){
        this.velocity = velocity;
        wrist = Wrist.getInstance();
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        wrist.setVelocityMode(true);
    }

    @Override
    public void execute() {
        // wrist.setWristVelocity(velocity.getAsDouble());
        wrist.setWristVelocityWristFeed(velocity.getAsDouble());
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
