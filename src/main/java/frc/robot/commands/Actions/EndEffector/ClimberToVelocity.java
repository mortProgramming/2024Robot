package frc.robot.commands.Actions.EndEffector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberToVelocity extends Command{
    private Climber climber;
    private DoubleSupplier leftVelocity;
    private DoubleSupplier rightVelocity; 


    public ClimberToVelocity(DoubleSupplier leftVelocity, DoubleSupplier rightVelocity){
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
        climber.setRightClimberVelocity(rightVelocity.getAsDouble());
        climber.setLeftClimberVelocity(leftVelocity.getAsDouble());
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
