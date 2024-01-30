package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberToVelocity extends Command{
    private Climber climber;
    private double velocity;

    public ClimberToVelocity(double velocity){
        this.velocity = velocity;
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
        climber.setClimberVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
