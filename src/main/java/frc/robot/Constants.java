/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class JoystickButtons {
        public static final int kRESET_DRIVE = 8;
        public static final int kVISION_ON = 2;
    }

    public static final class DriveConstants {
        public static final int kLEFT_MOTOR_1_PORT = 2;
        public static final int kLEFT_MOTOR_2_PORT = 3;
        public static final int kRIGHT_MOTOR_1_PORT = 4;
        public static final int kRIGHT_MOTOR_2_PORT = 5;
        public static final boolean kIS_DRIVE_INVERTED = true;
        
        public static final double kRAMP_RATE = 1.0;
        public static final int kCURRENT_LIMIT = 40;

        public static final double kGEARBOX_REDUCTION = (50.0/12.0) * (60.0/14.0);
        public static final double kTIRE_SIZE_IN = 7.9;
        public static final double kTIRE_SIZE_M = Units.inchesToMeters(kTIRE_SIZE_IN);
        public static final int kPULSE_PER_ROTATION = 1;
        public static final double kENCODER_DISTANCE_PER_PULSE_M = ((double) kPULSE_PER_ROTATION / kGEARBOX_REDUCTION) * (kTIRE_SIZE_M * Math.PI);
        //public static final double kENCODER_DISTANCE_PER_PULSE_M = (kTIRE_SIZE_M * Math.PI) / ((double) kPULSE_PER_ROTATION / kGEARBOX_REDUCTION);
        
        public static final double kTRACK_WIDTH_M = 0.64;

        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(kTRACK_WIDTH_M);

    }

    public static final class VisionConstants {
        public static final int kSERVO_SHOOTER_ANGLE = 0;
        public static final int kSERVO_DOWN_ANGLE = 0;
        public static final int kSERVO_UP_ANGLE = 90;
        public static final int kLIGHT_RELAY_PORT = 0;

        public static final String kVISION_TABLE_KEY = "Vision";
        public static final String kIS_LOADING_STATION_ALIGNED_KEY = "isLoadingStationAligned";
        public static final String kIS_HIGH_GOAL_ALIGNED_KEY = "isHighGoalAligned";
        public static final String kIS_VISION_ON_KEY = "isVisionOn";
        public static final String kVISION_OFFSET_KEY = "highGoalOffset";
        public static final String kVISION_DISTANCE_KEY = "highGoalDistance";

        public static final String kVISION_TAB_KEY = "Vision Table";

    }
}
