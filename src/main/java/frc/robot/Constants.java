// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class LedsProfile {
        public static final int id = 5;
        public static final int num_leds = 126;
    }

    public class VisionProfile {
        public static final String limelightName = "limelight";
        public static final int blueSpeakerPipeline     = 0;
        public static final int redSpeakerPipeline      = 1;
        public static final int posFromSpeakerPipeline  = 2;
        public static final double kp = 0.3;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
    }

    public static class SwerveProfile {
        // gains will be scaled by TunerConstants.maxSpeed
        public static class angularController {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
        };
        public static class driveController {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
        };
    };
}
