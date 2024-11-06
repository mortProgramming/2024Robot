package org.mort11.commands.actions.endeffector.pos;

import static org.mort11.config.constants.PhysicalConstants.Climber.*;

import org.mort11.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberToPos extends Command{
    private Climber climber;
    private double leftSetpoint;
    private double rightSetpoint;

    public ClimberToPos(double leftSetpoint, double rightSetpoint){
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;

        climber = Climber.getInstance();
        
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setLeftSetpoint(leftSetpoint);
        climber.setRightSetpoint(rightSetpoint);
    }

    @Override
    public void execute() {
        climber.setLeftSetpoint(leftSetpoint);
        climber.setRightSetpoint(rightSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SERVOLOCKTEST");

        climber.setLeftServo(SERVO_GLOBAL_LOCK_POS);
        climber.setRightServo(SERVO_GLOBAL_LOCK_POS);
    }

    @Override
    public boolean isFinished() {
        return climber.getLeftSetpoint() && climber.getRightSetpoint();
    }
}
