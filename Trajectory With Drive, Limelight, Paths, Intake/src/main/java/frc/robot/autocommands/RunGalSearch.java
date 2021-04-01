package frc.robot.autocommands;

import java.util.Queue;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCross;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LimeLightSearch;
import frc.robot.commands.RunPath;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.QueueSubsystem;

public class RunGalSearch extends SequentialCommandGroup {

    /**
     * Creates a new RunGalSearch
     * @param drive The drive subsytem that will be used
     */
    public RunGalSearch(DriveTrain drive, QueueSubsystem queue) {
        addCommands(
                    //Deploy the arm
                    new AutoCross(drive),
                    //Look for the Ball
                    //new LimeLightSearch(),
                    //Run Intake -- End When RunGalSearch is done
                    new ParallelCommandGroup(
                            new LimeLightSearch(),
                            new IntakeBalls(queue)
                        ),
                    //Run Selected Path
                    new RunPath(drive)
                    );
    }
}
