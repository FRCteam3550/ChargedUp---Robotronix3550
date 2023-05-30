package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class DefaultSwerveModule implements SwerveModule {
    private final DriveController driveController;
    private final SteerController steerController;

    public DefaultSwerveModule(DriveController driveController,
                               SteerController steerController) {
        this.driveController = driveController;
        this.steerController = steerController;
    }

    @Override
    public double getDrivePosition() {
        return driveController.getStatePositionM();
    }

    @Override
    public double getDriveVelocity() {
        return driveController.getStateVelocityMS();
    }

    @Override
    public ContinuousAngle getSteerAngle() {
        return steerController.getStateAngle();
    }

    public void configureShuffleboardContainer(ShuffleboardContainer container) {
        // container.addNumber("Current Velocity", driveController::getStateVelocityMS);
        // container.addNumber("Target Velocity", driveController::getReferenceSpeedMS);
        // container.addNumber("Absolute Encoder Angle", () -> steerController.getAbsoluteAngle().degrees());
        // container.addNumber("Current Angle", () -> steerController.getStateAngle().degrees());
        // container.addNumber("Target Angle", () -> steerController.getReferenceAngle().degrees());
    }

    @Override
    public void set(double drivePct, DiscreetAngle steerAngleRadians) {
        var steerSetPoint = SwerveModule.getSteerAngleAndDriveSign(steerAngleRadians, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setReferencePct(drivePct * steerSetPoint.driveSign);
    }

    @Override
    public void setMS(double driveMS, DiscreetAngle steerAngleRadians) {
        var steerSetPoint = SwerveModule.getSteerAngleAndDriveSign(steerAngleRadians, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setReferenceSpeedMS(driveMS * steerSetPoint.driveSign);
    }    
}
