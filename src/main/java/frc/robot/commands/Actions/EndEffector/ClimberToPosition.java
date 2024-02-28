package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Climber.LEFT_UNLOCK_POSITION;
import static frc.robot.utility.Constants.Climber.RIGHT_UNLOCK_POSITION;
import static frc.robot.utility.Constants.Climber.SERVO_GLOBAL_LOCK_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Wrist;

public class ClimberToPosition extends Command{
    private Climber climber;
    private double leftSetpoint;
    private double rightSetpoint;

    public ClimberToPosition(double leftSetpoint, double rightSetpoint){
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
        climber.setLeftServo(SERVO_GLOBAL_LOCK_POSITION);
        climber.setRightServo(SERVO_GLOBAL_LOCK_POSITION);


    }

    @Override
    public boolean isFinished() {
        return climber.nearLeftSetpoint() && climber.nearRightSetpoint();
    }
}
