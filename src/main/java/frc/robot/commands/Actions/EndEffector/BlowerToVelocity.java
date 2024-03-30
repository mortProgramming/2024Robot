package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class BlowerToVelocity extends Command {
    private Arm arm;

    private double speed;

    public BlowerToVelocity(double speed){
        arm = Arm.getInstance();
        addRequirements(arm);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        arm.setBlowerTarget(speed);
    }

    @Override
    public void execute() {
        arm.setBlowerTarget(speed);
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.setBlowerTarget(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
