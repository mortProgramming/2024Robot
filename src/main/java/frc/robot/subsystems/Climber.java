package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private static Climber climber;

    public Climber(){
        //setup
    }
    
    /**
     * 
     */
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
    public Climber getInstance(){
        if (climber==null){
            climber = new Climber();
        }
        return climber;
    }
    
}
