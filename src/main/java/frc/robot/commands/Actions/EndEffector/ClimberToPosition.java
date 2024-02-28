package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Wrist.WRIST_INTAKE_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ClimberToPosition extends Command{
    private Climber climber;
    private Wrist wrist;
    private double leftSetpoint;
    private double rightSetpoint;

    public ClimberToPosition(double leftSetpoint, double rightSetpoint){
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        climber = Climber.getInstance();
        wrist = Wrist.getInstance();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        System.out.println("COMMAND RUNNING"); 
        climber.setLeftSetPoint(leftSetpoint);
        climber.setRightSetPoint(rightSetpoint);
    }

    @Override
    public void execute() {
        System.out.println("before controller");
        climber.getLeftClimberMotor().set(climber.getLeftController().calculate(climber.getLeftClimberMotor().getEncoder().getPosition(), leftSetpoint));
        climber.getRightClimberMotor().set(climber.getRightController().calculate(climber.getRightClimberMotor().getEncoder().getPosition(), rightSetpoint));
        System.out.println("after controller");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        climber.setVelocityMode(true);
    }

    @Override
    public boolean isFinished() {
        return climber.nearLeftSetpoint() && climber.nearRightSetpoint();
    }
}
