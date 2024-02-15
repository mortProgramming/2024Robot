package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;


public class ArmToVelocity extends Command{
    private Arm arm;
    private DoubleSupplier velocity;

    public ArmToVelocity(DoubleSupplier velocity){
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
        // arm.setArmVelocity(velocity.getAsDouble());
        arm.setArmVelocityG(velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
