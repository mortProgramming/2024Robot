package org.mort11.commands.actions.endeffector;

import org.mort11.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;   

public class IntakeToVelocity extends Command {
    private Intake intake;

    private double speed;

    public IntakeToVelocity(double speed){
        this.speed = speed;

        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeVelocity(0);
    }

    @Override
    public void execute() {
        intake.setIntakeVelocity(speed);
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
