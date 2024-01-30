package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToVelocity extends Command{
    private Arm arm;
    private double velocity;

    public ArmToVelocity(double velocity){
        this.velocity = velocity;
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        arm.setArmVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
