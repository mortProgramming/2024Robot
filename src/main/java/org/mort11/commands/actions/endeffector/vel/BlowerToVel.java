package org.mort11.commands.actions.endeffector.vel;

import org.mort11.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class BlowerToVel extends Command {
    private Arm arm;

    private double speed;

    public BlowerToVel(double speed) {
        this.speed = speed;

        arm = Arm.getInstance();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setBlowerTarget(speed);
    }

    @Override
    public void execute() {
        arm.setBlowerTarget(speed);
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
