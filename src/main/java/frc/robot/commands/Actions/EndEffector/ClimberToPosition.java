package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.util.Constants.Climber.*;

public class ClimberToPosition extends Command{
    private Climber climber;
    private double target;

    public ClimberToPosition(double target){
        this.target = target;
        climber = Climber.getInstance();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setSetPoint(target);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return climber.nearSetpoint();
    }
}
