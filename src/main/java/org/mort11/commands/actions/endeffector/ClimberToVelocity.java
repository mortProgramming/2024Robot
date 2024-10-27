package org.mort11.commands.actions.endeffector;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberToVelocity extends Command{
    private Climber climber;
    private double leftVelocity;
    private double rightVelocity; 

    public ClimberToVelocity(double leftVelocity, double rightVelocity){
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;

        climber = Climber.getInstance();

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        climber.setVelocityMode(true);
        climber.setRightClimberVelocity(rightVelocity);
        climber.setLeftClimberVelocity(leftVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRightClimberVelocity(0);
        climber.setLeftClimberVelocity(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
