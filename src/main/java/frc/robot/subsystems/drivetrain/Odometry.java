package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.utils.camestimate.CameraPoseEstimator;
import frc.robot.utils.camestimate.LimelightVisionCameraPoseEstimator;
//import frc.robot.utils.camestimate.PhotonVisionCameraPoseEstimator;

public class Odometry {
    public static final Pose2d INITIAL_POSE = new Pose2d( 2, 2, Rotation2d.fromDegrees(180));
    private static final double SPEED_UPDATE_DURATION_THRESHOLD_S = 0.01; // 10ms

    
    //private final CameraPoseEstimator m_cameraPoseEstimator = new PhotonVisionCameraPoseEstimator(TrainingField.LAYOUT);
    private final CameraPoseEstimator m_cameraPoseEstimator = new LimelightVisionCameraPoseEstimator();
    private final SwerveDrivePoseEstimator m_odometry;
    private Pose2d m_previousPoseM = INITIAL_POSE;
    private Timer m_timer = new Timer();
    private double m_speedMS = 0;
    private double m_accelerationMS2 = 0;
    private double m_angularSpeedDS = 0;
    private double m_angularAccelerationDS2 = 0;
    
    public Odometry(SwerveDriveKinematics kinematics,
                    Rotation2d gyroAngle, 
                    SwerveModulePosition[] positions) {

        m_odometry = new SwerveDrivePoseEstimator(kinematics, gyroAngle, positions, INITIAL_POSE);

        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("x m", () -> getPoseM().getX());
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("y m", () -> getPoseM().getY());
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("rot d", () -> getPoseM().getRotation().getDegrees());
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("vit ms", () -> m_speedMS);
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("accel ms2", () -> m_accelerationMS2);
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("om ds", () -> m_angularSpeedDS);
        PilotShuffleboardLayout.ODO_LAYOUT.addDouble("alpha ds2", () -> m_angularAccelerationDS2);
        m_timer.start();
    }

    public void update(Rotation2d gyroAngle, 
                       SwerveModulePosition[] positions) {
        m_odometry.update(gyroAngle, positions);

        m_cameraPoseEstimator.setReferencePose(m_odometry.getEstimatedPosition());
        var maybeCameraEstimate = m_cameraPoseEstimator.getLatestEstimate();
        if (maybeCameraEstimate.isPresent()) {
            var cameraEstimate = maybeCameraEstimate.get();
            m_odometry.addVisionMeasurement(
                cameraEstimate.poseM,
                cameraEstimate.timestampS
            );
        } 

        double durationS = m_timer.get();

        // Peut-être que pour une raison ou une autre, c'est appelé trop vite, rendant la division inopérante.
        if (durationS > SPEED_UPDATE_DURATION_THRESHOLD_S) {
            double distanceM = getPoseM().getX() - m_previousPoseM.getX();
            var newSpeedMS = distanceM / durationS;
            m_accelerationMS2 = (newSpeedMS - m_speedMS) / durationS;
            m_speedMS = newSpeedMS;
            
            double diffAngleDeg = getPoseM().getRotation().getDegrees() - m_previousPoseM.getRotation().getDegrees();
            var newAngularSpeedDS = diffAngleDeg / durationS;
            m_angularAccelerationDS2 = (newAngularSpeedDS - m_angularSpeedDS) / durationS;
            m_angularSpeedDS = newAngularSpeedDS;

            m_timer.reset();
            m_previousPoseM = getPoseM();
        }
    }

    public Pose2d getPoseM () {
        return m_odometry.getEstimatedPosition();
    }

    public void setPose(Rotation2d gyroAngle, 
                        SwerveModulePosition[] positions,
                        Pose2d actualPoseM) {
        m_odometry.resetPosition(gyroAngle, positions, actualPoseM);
    }
}
