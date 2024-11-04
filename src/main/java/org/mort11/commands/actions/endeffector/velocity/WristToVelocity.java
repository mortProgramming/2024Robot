package org.mort11.commands.actions.endeffector.velocity;

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
