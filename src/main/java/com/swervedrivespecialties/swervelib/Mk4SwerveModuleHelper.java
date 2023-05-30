package com.swervedrivespecialties.swervelib;

import com.swervedrivespecialties.swervelib.ctre.CANCoderAbsoluteEncoder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveController;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerController;
import com.swervedrivespecialties.swervelib.rev.NeoDriveConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoDriveController;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoSteerController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk4SwerveModuleHelper {
    private static final int CAN_CODER_READING_UPDATE_PERIOD_MS = 100;
    private Mk4SwerveModuleHelper() {
    }

    private static DriveController getFalcon500DriveController(int motorCANId,
                                                            GearRatio moduleConfig) {
        var driveConfiguration = new Falcon500DriveConfiguration(motorCANId);
        return new Falcon500DriveController(driveConfiguration, moduleConfig);
    }

    private static SteerController getFalcon500SteerController(int motorCANId,
                                                               int steerEncoderCANId,
                                                               DiscreetAngle steerAlignAngle,
                                                               GearRatio moduleConfig) {
        var absoluteEncoder = new CANCoderAbsoluteEncoder(steerEncoderCANId, steerAlignAngle, CAN_CODER_READING_UPDATE_PERIOD_MS);
        var steerConfiguration = new Falcon500SteerConfiguration(motorCANId, absoluteEncoder)
                .withPidConstants(0.75, 0, 2);
        return new Falcon500SteerController(steerConfiguration, moduleConfig);
    }

    private static DriveController getNeoDriveController(int motorCANId,
                                                      GearRatio moduleConfig) {
        var driveConfiguration = new NeoDriveConfiguration(motorCANId);
        return new NeoDriveController(driveConfiguration, moduleConfig);
    }

    private static SteerController getNeoSteerController(int motorCANId,
                                                         int steerEncoderCANId,
                                                         DiscreetAngle steerAlignAngle,
                                                         GearRatio moduleConfig) {
        var absoluteEncoder = new CANCoderAbsoluteEncoder(steerEncoderCANId, steerAlignAngle, CAN_CODER_READING_UPDATE_PERIOD_MS);
        var steerConfiguration = new NeoSteerConfiguration(motorCANId, absoluteEncoder)
                .withPidConstants(0.75, 0, 2);
        return new NeoSteerController(steerConfiguration, moduleConfig);
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        var result = new DefaultSwerveModule(
                getFalcon500DriveController(driveMotorPort, gearRatio),
                getFalcon500SteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );

        result.configureShuffleboardContainer(container);

        return result;
    }

    /**
     * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        return new DefaultSwerveModule(
                getFalcon500DriveController(driveMotorPort, gearRatio),
                getFalcon500SteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        var result =  new DefaultSwerveModule(
                getNeoDriveController(driveMotorPort, gearRatio),
                getNeoSteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );

        result.configureShuffleboardContainer(container);

        return result;
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        return new DefaultSwerveModule(
                getNeoDriveController(driveMotorPort, gearRatio),
                getNeoSteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            ShuffleboardLayout container,
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        var result = new DefaultSwerveModule(
                getFalcon500DriveController(driveMotorPort, gearRatio),
                getNeoSteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );

        result.configureShuffleboardContainer(container);

        return result;
    }

    /**
     * Creates a Mk4 swerve module that uses a Falcon 500 for driving and a NEO for steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500Neo(
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        return new DefaultSwerveModule(
                getFalcon500DriveController(driveMotorPort, gearRatio),
                getNeoSteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            ShuffleboardLayout container,
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        var result = new DefaultSwerveModule(
                getNeoDriveController(driveMotorPort, gearRatio),
                getFalcon500SteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );

        result.configureShuffleboardContainer(container);

        return result;
    }

    /**
     * Creates a Mk4 swerve module that uses a NEO for driving and a Falcon 500 for steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeoFalcon500(
            int driveMotorPort,
            int steerMotorPort,
            GearRatio gearRatio,
            int steerEncoderPort,
            DiscreetAngle steerAlignAngle
    ) {
        return new DefaultSwerveModule(
                getNeoDriveController(driveMotorPort, gearRatio),
                getFalcon500SteerController(steerMotorPort, steerEncoderPort, steerAlignAngle, gearRatio)
        );
    }
}
