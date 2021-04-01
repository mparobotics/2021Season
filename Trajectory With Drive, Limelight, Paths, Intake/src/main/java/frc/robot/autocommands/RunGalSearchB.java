package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCross;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.LimeLightSearch;
import frc.robot.commands.RunPathB;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.QueueSubsystem;

public class RunGalSearchB extends SequentialCommandGroup {

    /**
     * Creates a new RunGalSearch
     * @param drive The drive subsytem that will be used
     */
    public RunGalSearchB(DriveTrain drive, QueueSubsystem queue) {
        addCommands(
                    //Deploy the arm
                    new AutoCross(drive),
                    new IntakeBalls(queue),
                    new LimeLightSearch(),
                    //Run Selected Path
                    new RunPathB(drive)
                    );
    }
}
