// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.util.struct.Struct;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Controller {
    public static final int kDriverControllerPort = 0;

    public static final double kJoystickDeadband = 0.05;
  }

  public static class AprilTags {
    public static final class Scorers {
      public static final class RedAlliance {
        public static final int kScorerFrontOffset = 9; /// From red driver station, infront of mid station
        public static final int kScorerFront = 10; /// From red driver station, infront of mid station, offseted.
      }

      public static final class BlueAlliance {
        public static final int kScorerFrontOffset = 25; /// From blue driver station, infront of mid station
        public static final int kScorerFront = 26; /// From blue driver station, infront of mid station, offseted.
      }
    }
  }

  /**
   * DRIVETRAIN IDS:
   * FL - Krakens 1, 2, cancoder 11
   * FR - Krakens 3, 4, cancoder 12
   * BL - Krakens 5, 6, cancoder 14
   * BR - Krakens 7, 8, cancoder 15
   */

  /**
   * Motors
   */

  public static class Motors {
    public static class Swerve {
      // the just in case

      public static final int kFrontLeftDriveID = 1;
      public static final int kFrontLeftSteerID = 2;
      public static final int kFrontLeftCancoderID = 11;

      public static final int kFrontRightDriveID = 3;
      public static final int kFrontRightSteerID = 4;
      public static final int kFrontRightCancoderID = 12;

      public static final int kBackLeftDriveID = 5;
      public static final int kBackLeftSteerID = 6;
      public static final int kBackLeftCancoderID = 14;

      public static final int kBackRightDriveID = 7;
      public static final int kBackRightSteerID = 8;
      public static final int kBackRightCancoderID = 15;

      public static class PIDValues {
        public static final double kAimP = 0;
        public static final double kAimI = 0;
        public static final double kAimD = 0;
      }
    }

    public static class Mortar {

      public static final int kSeathID = 0;
      public static final int kFlywheelID = 0;

      public static final double kSeathSoftLimit = 0;
      public static final double kFlywheelSoftLimit = 0;

      public static class PIDValues {
        public static final double kSeathP = 0;
        public static final double kSeathI = 0;
        public static final double kSeathD = 0;
      }

      public static class Hardstop {

        public static final double kSeathFront = 0;
        public static final double kSeathBack = 0;
      }
    }
  }
}