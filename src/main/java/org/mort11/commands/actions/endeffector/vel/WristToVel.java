package org.mort11.commands.actions.endeffector.vel;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Wrist;

public class WristToVel extends Command{
    private Wrist wrist;

    private DoubleSupplier velocity;

    public WristToVel(DoubleSupplier velocity){
        this.velocity = velocity;

        wrist = Wrist.getInstance();

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setWristVelocity(velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setWristVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
