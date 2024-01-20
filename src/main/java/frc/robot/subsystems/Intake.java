package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intake;

    public Intake() {

    }

    public void init() {
    //add motor initialization
    }

    /**
     * 
     */
    @Override
    public void periodic() {
      // This method will be called once per scheduler run

    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    /**
     * 
     */
    public void intake(){
        //Motor.setpercentOutput(speed);
    }

    /**
     * 
     */
    public void outtake(){
        //Motor.setpercentOutput(-speed);
    }

    /**
     * 
     * @return
     */
    public Intake getInstance(){
        if (intake==null){
            intake = new Intake();
        }
        return intake;
    }
}
