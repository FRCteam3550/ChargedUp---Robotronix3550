package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class HolonomicDriveControllerWithTelemetry implements Sendable {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private boolean m_firstRun = true;
  private Pose2d m_lastRef = new Pose2d();
  private Pose2d m_lastOutput = new Pose2d();
  private Pose2d m_lastActual = new Pose2d();
  private double m_xFF = 0;
  private double m_yFF = 0;
  private double m_thetaFF = 0;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  public HolonomicDriveControllerWithTelemetry(
      PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));

    SendableRegistry.add(this, "Swerve Controller");
    SmartDashboard.putData(this);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  public void reset() {
    m_firstRun = true;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param trajectoryPose The desired trajectory pose, as sampled for the current timestep.
   * @param desiredLinearVelocityMetersPerSecond The desired linear velocity.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredHeading) {

        SmartDashboard.putNumber("Actual 2 x", currentPose.getX());
        SmartDashboard.putNumber("Actual 2 y", currentPose.getY());
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_firstRun = false;
    }

    // Calculate feedforward velocities (field-relative).
    m_xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    m_yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    m_thetaFF =
        m_thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    m_poseError = trajectoryPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(m_xFF, m_yFF, m_thetaFF, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());

    m_lastRef = trajectoryPose;
    m_lastActual = currentPose;
    m_lastOutput = new Pose2d(m_xFF + xFeedback, m_yFF + yFeedback, Rotation2d.fromRadians(m_thetaFF));

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        m_xFF + xFeedback, m_yFF + yFeedback, m_thetaFF, currentPose.getRotation());
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param desiredState The desired trajectory pose, as sampled for the current timestep.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(getClass().getSimpleName());
    
    // // Error
    // builder.addDoubleProperty("x error m", () -> m_poseError.getX(), null);
    // builder.addDoubleProperty("y error m", () -> m_poseError.getY(), null);
    // builder.addDoubleProperty("theta error def", () -> m_rotationError.getDegrees(), null);

    // // Position set point / reference
    // builder.addDoubleProperty("x ref m", () -> m_lastRef.getX(), null);
    // builder.addDoubleProperty("y ref m", () -> m_lastRef.getY(), null);
    // builder.addDoubleProperty("theta ref deg", () -> m_lastRef.getRotation().getDegrees(), null);

    // // Velocity set point / reference
    // builder.addDoubleProperty("vx ref ms", () -> m_xFF, null);
    // builder.addDoubleProperty("vy ref ms", () -> m_yFF, null);
    // builder.addDoubleProperty("omega ref degs", () -> Math.toDegrees(m_thetaFF), null);

    // // Output
    // builder.addDoubleProperty("vx output ms", () -> m_lastOutput.getX(), null);
    // builder.addDoubleProperty("vy output ms", () -> m_lastOutput.getY(), null);
    // builder.addDoubleProperty("omega output degs", () -> m_lastOutput.getRotation().getDegrees(), null);

    // // Actual
    // builder.addDoubleProperty("x actual m", () -> m_lastActual.getX(), null);
    // builder.addDoubleProperty("y actual m", () -> m_lastActual.getY(), null);
    // builder.addDoubleProperty("theta actual deg", () -> m_lastActual.getRotation().getDegrees(), null);
  }  
}
