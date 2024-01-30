package frc.robot.commands.Actions.EndEffector;

import static frc.robot.util.Constants.Intake.*;

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
