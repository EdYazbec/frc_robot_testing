package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100)
        .withKI(0)
        .withKD(1)
        .withKS(0)
        .withKV(0)
        .withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.25)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0)
        .withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 30.0;

    public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static final double maxAngularRate = 1 * Math.PI; // 1/4 of a rotation per second max angular velocity

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 1;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 8.142857142857142;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 4;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(kPigeonId)
        .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kSteerGearRatio)
        .withWheelRadius(kWheelRadiusInches)
        .withSlipCurrent(kSlipCurrentA)
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
        .withSteerInertia(kSteerInertia)
        .withDriveInertia(kDriveInertia)
        .withSteerFrictionVoltage(kSteerFrictionVoltage)
        .withDriveFrictionVoltage(kDriveFrictionVoltage)
        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
        .withCouplingGearRatio(kCoupleRatio)
        .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 31;
    private static final int kFrontLeftSteerMotorId = 32;
    private static final int kFrontLeftEncoderId = 33;
    private static final double kFrontLeftEncoderOffset = -0.275146484375;

    private static final double kFrontLeftXPosInches = 7.5;
    private static final double kFrontLeftYPosInches = 7.5;

    // Front Right
    private static final int kFrontRightDriveMotorId = 21;
    private static final int kFrontRightSteerMotorId = 22;
    private static final int kFrontRightEncoderId = 23;
    private static final double kFrontRightEncoderOffset = 0.175537109375;

    private static final double kFrontRightXPosInches = 7.5;
    private static final double kFrontRightYPosInches = -7.5;

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 3;
    private static final double kBackLeftEncoderOffset = -0.02880859375;

    private static final double kBackLeftXPosInches = -7.5;
    private static final double kBackLeftYPosInches = 7.5;

    // Back Right
    private static final int kBackRightDriveMotorId = 11;
    private static final int kBackRightSteerMotorId = 12;
    private static final int kBackRightEncoderId = 13;
    private static final double kBackRightEncoderOffset = 0.4638671875;

    private static final double kBackRightXPosInches = -7.5;
    private static final double kBackRightYPosInches = -7.5;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
        kFrontLeftSteerMotorId, 
        kFrontLeftDriveMotorId, 
        kFrontLeftEncoderId, 
        kFrontLeftEncoderOffset, 
        Units.inchesToMeters(kFrontLeftXPosInches), 
        Units.inchesToMeters(kFrontLeftYPosInches), 
        kInvertLeftSide
    );

    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
        kFrontRightSteerMotorId, 
        kFrontRightDriveMotorId, 
        kFrontRightEncoderId, 
        kFrontRightEncoderOffset, 
        Units.inchesToMeters(kFrontRightXPosInches), 
        Units.inchesToMeters(kFrontRightYPosInches), 
        kInvertRightSide
    );

    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
        kBackLeftSteerMotorId, 
        kBackLeftDriveMotorId, 
        kBackLeftEncoderId, 
        kBackLeftEncoderOffset, 
        Units.inchesToMeters(kBackLeftXPosInches), 
        Units.inchesToMeters(kBackLeftYPosInches), 
        kInvertLeftSide
    );

    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
        kBackRightSteerMotorId, 
        kBackRightDriveMotorId, 
        kBackRightEncoderId, 
        kBackRightEncoderOffset, 
        Units.inchesToMeters(kBackRightXPosInches), 
        Units.inchesToMeters(kBackRightYPosInches), 
        kInvertRightSide
    );

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(
            DrivetrainConstants, 
            FrontLeft,
            FrontRight, 
            BackLeft, 
            BackRight
    );

    private static final Translation2d frontLeftLocation  = new Translation2d(kFrontLeftXPosInches  * 0.0254, kFrontLeftYPosInches  * 0.0254);
    private static final Translation2d frontRightLocation = new Translation2d(kFrontRightXPosInches * 0.0254, kFrontRightYPosInches * 0.0254);
    private static final Translation2d backLeftLocation   = new Translation2d(kBackLeftXPosInches   * 0.0254, kBackLeftYPosInches   * 0.0254);
    private static final Translation2d backRightLocation  = new Translation2d(kBackRightXPosInches  * 0.0254, kBackRightYPosInches  * 0.0254);
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        TunerConstants.frontLeftLocation,
        TunerConstants.frontRightLocation,
        TunerConstants.backLeftLocation,
        TunerConstants.backRightLocation
    );
}
