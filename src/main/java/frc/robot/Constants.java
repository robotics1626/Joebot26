// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class Operator {

    public static final int kDriverControllerPort = 0;
  }

  /**
   * Motors
   */
  public static class Motors {
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
