package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.SelfReplacingCommand;

public class TrajectoryFollowing {
    private static final double MAX_VELOCITY_MS = 2; //6.28
    private static final double MAX_ACCELERATION_M_S2 = 1; //3.14
    private static final double MAX_VELOCITY_DEG_MS = 100; //6.28
    private static final double MAX_ACCELERATION_DEG_MS2 = 100; //3.14

    private static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_VELOCITY_MS, MAX_ACCELERATION_M_S2);

    private static final double K_P_TRANS = 0.4; //1
    private static final double K_I_TRANS = 0.1;
    private static final double K_D_TRANS = 0;
    
    private static final double K_P_ROT = 1; //1
    private static final double K_I_ROT = 0;
    private static final double K_D_ROT = 0;

    private static final double TRANSLATION_TOLERANCE_M = 0.01;
    private static final Rotation2d ROTATION_TOLERANCE_D = Rotation2d.fromDegrees(2);
    private static final Pose2d TOLERANCE = new Pose2d(TRANSLATION_TOLERANCE_M, TRANSLATION_TOLERANCE_M, ROTATION_TOLERANCE_D);

    private final PIDController m_xController = new PIDController(K_D_TRANS, K_I_TRANS, K_P_TRANS);
    private final PIDController m_yController = new PIDController(K_D_ROT, K_I_ROT, K_P_ROT);
    private final TrapezoidProfile.Constraints m_trapezoidProfile = new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_MS, MAX_ACCELERATION_DEG_MS2);
    private final ProfiledPIDController m_thetaController = new ProfiledPIDController(K_P_ROT, K_I_ROT, K_D_ROT, m_trapezoidProfile);

    private final SwerveDriveKinematics m_kinematics;
    private final Supplier<Pose2d> m_odometry;
    private final HolonomicDriveControllerWithTelemetry m_controller = new HolonomicDriveControllerWithTelemetry(m_xController, m_yController, m_thetaController);
    private final Consumer<SwerveModuleState[]> m_setOutput;

    private final Subsystem m_subsystem;
    private final Consumer<Trajectory> m_displayTrajectory;

    public TrajectoryFollowing(
        SwerveDriveKinematics kinematics, 
        Supplier<Pose2d> odometry, 
        Consumer<SwerveModuleState[]> setOutput, 
        Subsystem subsystem,
        Consumer<Trajectory> displayTrajectory
        ) {
        this.m_kinematics = kinematics;
        this.m_odometry = odometry;
        this.m_setOutput = setOutput;
        this.m_subsystem = subsystem;
        this.m_displayTrajectory = displayTrajectory;

        m_xController.setIntegratorRange(-.20, .20);
        m_yController.setIntegratorRange(-.20, .20);

        m_controller.setTolerance(TOLERANCE);
    }

    public Command goToLocation(Pose2d startM, Pose2d locationM, Rotation2d desiredRotation) {
        return new SelfReplacingCommand(
            () -> {
                var trajectory = TrajectoryGenerator.generateTrajectory(startM, List.of(), locationM, TRAJECTORY_CONFIG); // m_odometry.get()

                m_displayTrajectory.accept(trajectory);
        
                m_controller.reset();
        
                return new SwerveControllerCommandWithTelemetry(
                    trajectory, 
                    m_odometry, 
                    m_kinematics, 
                    m_controller, 
                    () -> desiredRotation,
                    m_setOutput, 
                    m_subsystem
                );
            },
            m_subsystem
        );
    }

    public Command follow(Trajectory trajectory, Rotation2d desiredEndHeading) {
        return new SwerveControllerCommandWithTelemetry(
            trajectory, 
            m_odometry, 
            m_kinematics, 
            m_controller,
            () -> desiredEndHeading,
            m_setOutput, 
            m_subsystem
        ).beforeStarting(() -> {
            m_displayTrajectory.accept(trajectory);        
            m_controller.reset();
        });
    }    
}
