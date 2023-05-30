package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PilotShuffleboardLayout;

public class Cameras extends SubsystemBase{
    enum CurrentCamera {
        CAMERAUP,
        CAMERADOWN
    }

    CurrentCamera m_current = CurrentCamera.CAMERADOWN;

    private static int DOWN_CAMERA_USB_PORT = 0;
    private static int UP_CAMERA_USB_PORT = 1;
    private static int H_RESOLUTION_PIX = 320;
    private static int V_RESOLUTION_PIX = 240;
    private static int FPS = 30;

    private final UsbCamera m_cameraDown;
    private final UsbCamera m_cameraUp;
    private final VideoSink m_server;

    public Cameras() {
        //m_cameraDown = CameraServer.startAutomaticCapture("Down", DOWN_CAMERA_USB_PORT);
       // m_cameraUp = CameraServer.startAutomaticCapture("Up", UP_CAMERA_USB_PORT);
        m_cameraDown = new UsbCamera("Down", DOWN_CAMERA_USB_PORT);
        //CameraServerSharedStore.getCameraServerShared().reportUsbCamera(m_cameraDown.getHandle());
        CameraServer.addCamera(m_cameraDown);

        m_cameraUp = new UsbCamera("Up", UP_CAMERA_USB_PORT);
        //CameraServerSharedStore.getCameraServerShared().reportUsbCamera(m_cameraUp.getHandle());
        CameraServer.addCamera(m_cameraUp);

        m_server = CameraServer.addServer("MainCameraServer");
        //m_server = CameraServer.getServer();
        m_server.setSource(m_cameraDown);
        m_cameraDown.setVideoMode(PixelFormat.kMJPEG, H_RESOLUTION_PIX, V_RESOLUTION_PIX, FPS);
        m_cameraUp.setVideoMode(PixelFormat.kMJPEG, H_RESOLUTION_PIX, V_RESOLUTION_PIX, FPS);

        adjustVideoMode(m_cameraDown);
        adjustVideoMode(m_cameraUp);

        // May do: HUD indicators for the pilot: https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html

        PilotShuffleboardLayout.PILOT_TAB.add("Video" ,m_server.getSource()).withSize(3, 3).withPosition(8, 0);
    }
    
    private void adjustVideoMode(UsbCamera camera) {
        var succeeded = false;

        while (!succeeded) {
            succeeded = camera.setVideoMode(PixelFormat.kMJPEG, H_RESOLUTION_PIX, V_RESOLUTION_PIX, FPS);
            try {
                Thread.sleep(20);
            } catch(InterruptedException ie) {
                // Do nothing
            }
        }
    }

    public Command switchCameraDown() {
        return runOnce(() -> {
            m_cameraUp.setConnectionStrategy(ConnectionStrategy.kForceClose);
            m_cameraDown.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            m_server.setSource(m_cameraDown);
            m_current = CurrentCamera.CAMERADOWN;
        }).ignoringDisable(true);
    }

    public Command switchCameraUp() {
        return runOnce(() -> {
            m_cameraUp.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            m_cameraDown.setConnectionStrategy(ConnectionStrategy.kForceClose);
            m_server.setSource(m_cameraUp);
            m_current = CurrentCamera.CAMERAUP;
        }).ignoringDisable(true);
    }

    public Command cameraToggle() {
        return Commands.either(switchCameraUp(), switchCameraDown(),() -> m_current == CurrentCamera.CAMERADOWN)
        .withName("CamerToggeling")
        .ignoringDisable(true);
    }
}
