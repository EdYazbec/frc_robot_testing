// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.TunerConstants;

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
        public static PIDConstants translationControllerConstants = new PIDConstants(2, 0, 0);
        public static PIDController translationController = new PIDController(
            translationControllerConstants.kP,
            translationControllerConstants.kI,
            translationControllerConstants.kD
        );
        public static double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        public static double maxAngularRate = Math.PI;

        public static PIDConstants angularControllerConstants = new PIDConstants(0.3, 0, 0);
        public static PIDController angularController = new PIDController(
            angularControllerConstants.kP, 
            angularControllerConstants.kI,
            angularControllerConstants.kD
        );
    };
}
