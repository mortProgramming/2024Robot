package frc.robot.commands.Actions.EndEffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberToVelocity extends Command{
    private Climber climber;
    private DoubleSupplier velocity;
    private boolean leftOrRightClimber;

    public ClimberToVelocity(DoubleSupplier velocity, boolean leftOrRightClimber){
        this.velocity = velocity;
        this.leftOrRightClimber = leftOrRightClimber;
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
        if(leftOrRightClimber) {
            climber.setRightClimberVelocity(velocity.getAsDouble());
        }
        else {
            climber.setLeftClimberVelocity(velocity.getAsDouble());

        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setLeftClimberVelocity(0);
        climber.setRightClimberVelocity(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
