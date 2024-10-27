package org.mort11.commands.actions.endeffector;

import static org.mort11.configuration.constants.PhysicalConstants.Climber.LEFT_UNLOCK_POS;
import static org.mort11.configuration.constants.PhysicalConstants.Climber.RIGHT_UNLOCK_POS;
import static org.mort11.configuration.constants.PhysicalConstants.Climber.SERVO_GLOBAL_LOCK_POS;

import org.mort11.subsystems.Climber;
import org.mort11.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
        climber.setLeftSetPoint(leftSetpoint);
        climber.setRightSetPoint(rightSetpoint);
       
    }

    @Override
    public void execute() {
    
        climber.getLeftClimberMotor().set(climber.getLeftController().calculate(climber.getLeftClimberMotor().getEncoder().getPosition(), leftSetpoint));
        climber.getRightClimberMotor().set(climber.getRightController().calculate(climber.getRightClimberMotor().getEncoder().getPosition(), rightSetpoint));
        
        
    }

    @Override
    public void end(boolean interrupted) {
        climber.setVelocityMode(true);
        System.out.println("SERVOLOCKTEST");
        climber.setLeftServo(SERVO_GLOBAL_LOCK_POS);
        climber.setRightServo(SERVO_GLOBAL_LOCK_POS);


    }

    @Override
    public boolean isFinished() {
        return climber.nearLeftSetpoint() && climber.nearRightSetpoint();
    }
}
