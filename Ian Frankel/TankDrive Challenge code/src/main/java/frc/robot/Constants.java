/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * global constants are declared and initailized here
 */
public final class Constants {

    /**
     * global constants used in manipulating the drive train
     * (motor controller ids, conversion rate from ticks to feet for encoders, etc.)
     */
    public final class DriveConstants {
        public static final int FALCON_FR_ID = 44;
        public static final int FALCON_BR_ID = 41;
        public static final int FALCON_FL_ID = 43;
        public static final int FALCON_BL_ID = 42;

        public static final double TIC_FT = ((Math.PI)/2014)/10.75;

        public static final double DRIVE_P = 4;
        public static final double DRIVE_I = 1;

    }

    /**
     * global constants used in the queue (intake, indexer, conveyor)
     * (motor controller ids, speeds, etc.) 
     */


    /**
     * global constants used in the shooter
     * (motor controller ids, pid values/ motor speeds, etc.) 
     */

    /**
     * global constants used in the climber
     * (motor controller ids, pid values/ motor speeds, etc.) 
     */

    /**
     * global constants used in limelight
     * (idk)
     */

    /**
     * global constants used in user input/ output 
     * (xbox controller port id, etc.)
     */
    public final class OIConstants {
        //public static final int XBOX_ID = 0;
        //public static final int HELMS_ID = 1;
        public static final int left_Joystick = 0;
        public static final int right_Joystick = 1;
    }
}
