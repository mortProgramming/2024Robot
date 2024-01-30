package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends Command{
    private Arm arm;
    private double target;

    public ArmToPosition(double target){
        this.target = target;
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setSetPoint(target);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return arm.nearSetpoint();
    }
}
