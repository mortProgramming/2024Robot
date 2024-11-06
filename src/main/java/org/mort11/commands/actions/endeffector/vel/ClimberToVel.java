package org.mort11.commands.actions.endeffector.vel;

import org.mort11.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberToVel extends Command{
    private Climber climber;
    
    private double leftVelocity;
    private double rightVelocity; 

    public ClimberToVel(double leftVelocity, double rightVelocity){
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;

        climber = Climber.getInstance();

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setRightVelocity(rightVelocity);
        climber.setLeftVelocity(leftVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRightVelocity(0);
        climber.setLeftVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
