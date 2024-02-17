package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;   

public class IntakeToVelocity extends Command {
    private Intake intake;

    private double speed;

    public IntakeToVelocity(double speed){
        intake = Intake.getInstance();
        addRequirements(intake);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setIntakeVelocity(0);
    }

    @Override
    public void execute() {
        if(speed > 0)
            intake.setIntakeVelocity(speed);
        else
            intake.setOuttakeVelocity(-speed);
        
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
