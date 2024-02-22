package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberToPosition extends Command{
    private Climber climber;
    private double target;
    private boolean leftOrRightClimber;

    public ClimberToPosition(double target, boolean leftOrRightClimber){
        this.target = target;
        climber = Climber.getInstance();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if(leftOrRightClimber) {
            climber.setRightSetPoint(target);

        }
        else {
            climber.setLeftSetPoint(target);
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(leftOrRightClimber) {
            return climber.nearRightSetpoint();

        }
        else {
            return climber.nearLeftSetpoint();
        }
    }
}
