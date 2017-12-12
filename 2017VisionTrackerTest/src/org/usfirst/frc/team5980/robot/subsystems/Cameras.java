package org.usfirst.frc.team5980.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Cameras extends Subsystem implements Runnable {
    Thread tracking;
    Mat source, mask, hsv, hierarchy;
    Scalar lowerHSV, upperHSV;
    UsbCamera camera;
    Object visionLock = new Object();
    boolean trackingOn = false;
    double targetX = Double.NaN;
    double targetY = Double.NaN;
    CvSink currentSink;
    UsbCamera currentCam;
    
    public Cameras () {
        source = new Mat();
        mask = new Mat();
        hsv = new Mat();
        hierarchy = new Mat();
        lowerHSV = new Scalar(55,100,100);//75
        upperHSV = new Scalar(100,255,255);//160
        tracking = new Thread(this);
        camera = new UsbCamera("front", 0);
        currentCam = camera;
        CameraServer.getInstance().addCamera(currentCam);
        currentSink = CameraServer.getInstance().getVideo(currentCam);
    }
    
    public void trackingOn(boolean on) {
        synchronized(visionLock) {
                trackingOn = on;
                if(on == false) {
                        targetX = Double.NaN;
                        targetY = Double.NaN;
                }
        }
    }
    
    public boolean isTrackingOn() {
        synchronized(visionLock) {
                return trackingOn;
        }
    }

    public void startCamera() {
        tracking.start();
    }
    
    public void run() {
        int localWidth = 320;
        int localHeight = 240;
        camera.setResolution(localWidth, localHeight);//sets resolution (7mb/s bandwidth limit!)
        camera.setPixelFormat(PixelFormat.kYUYV);//sets the pixel format for fast camera switching (C920)
        camera.setFPS(20);//sets fps
        camera.setBrightness(20);
        
        CvSource outputStream = CameraServer.getInstance().putVideo("Vision",  localWidth, localHeight);//creates a stream to the SmartDashboard
        while(true) {
            synchronized(visionLock) {
                currentSink.grabFrame(source);
            }
            outputStream.putFrame(source);
        }
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        
    }
}

