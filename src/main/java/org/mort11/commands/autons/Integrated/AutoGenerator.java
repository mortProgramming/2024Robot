package org.mort11.commands.autons.Integrated;


import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGenerator extends SequentialCommandGroup{
    int pathIndex;

    public AutoGenerator(String autoName, Command... otherCommand){

        List <PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);

        SequentialCommandGroup gimmeMyAuton;

        gimmeMyAuton = new SequentialCommandGroup();

        int greaterLength = (otherCommand.length > paths.size() ? otherCommand.length : paths.size());

        for (int i = 0; i < greaterLength; i++){
        
            if(otherCommand.length < i && paths.size() >= i) {
                gimmeMyAuton.andThen(
                    AutoBuilder.followPath(paths.get(i))
                );
            }

            else if(otherCommand.length >= i && paths.size() < i) {
                gimmeMyAuton.andThen(
                    otherCommand[i]
                );
            }

            else {
                gimmeMyAuton.andThen(
                    otherCommand[i],
                    AutoBuilder.followPath(paths.get(i))
                );
            }
        }

        new SequentialCommandGroup(

            gimmeMyAuton
    
        );

        
    }


}