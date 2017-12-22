package org.usfirst.frc.team5980.robot.subsystems;

import java.util.Date;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.*;

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
    Mat source, mask, hsv, hierarchy, gray;
    Scalar lowerHSV, upperHSV;
    UsbCamera camera;
    Object visionLock = new Object();
    boolean trackingOn = false;
    double targetX = Double.NaN;
    double targetY = Double.NaN;
    CvSink currentSink;
    UsbCamera currentCam;
    CvSource outputStream;
    int localWidth = 320;
    int localHeight = 240;
    
    public Cameras() {
        source = new Mat();
        mask = new Mat();
        hsv = new Mat();
        hierarchy = new Mat();
        gray = new Mat();
        lowerHSV = new Scalar(55, 100, 100);// 75
        upperHSV = new Scalar(100, 255, 255);// 160
        tracking = new Thread(this);
        
        /*
        camera = new UsbCamera("front", 0);
        currentCam = camera;
        CameraServer.getInstance().addCamera(currentCam);
        currentSink = CameraServer.getInstance().getVideo(currentCam);
        */
        
        camera = CameraServer.getInstance().startAutomaticCapture();
        //camera.setResolution(localWidth, localHeight);
        currentSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Vision", localWidth, localHeight);// creates a
        
    }

    public void trackingOn(boolean on) {
        synchronized (visionLock) {
            trackingOn = on;
            if (on == false) {
                targetX = Double.NaN;
                targetY = Double.NaN;
            }
        }
    }

    public boolean isTrackingOn() {
        synchronized (visionLock) {
            return trackingOn;
        }
    }

    public void startCamera() {
        tracking.start();
    }

    public void run() {
        
        camera.setResolution(localWidth, localHeight);// sets resolution (7mb/s bandwidth limit!)
        
        camera.setPixelFormat(PixelFormat.kYUYV);// sets the pixel format for fast camera switching (C920)
        camera.setFPS(20);// sets fps
        camera.setBrightness(20);
        camera.setExposureManual(1);
        
        
                                                                                                       // SmartDashboard
        while (true) {
            // synchronized(visionLock) {
            //System.out.println(source);
            //long frameTime = currentSink.grabFrame(source, 2.0);
            //camera.setResolution(localWidth, localHeight);
            currentSink.grabFrame(source);
            
            if(source.empty()) {
                System.out.println("is empty");
            } else {
                Imgproc.cvtColor(source, hsv, Imgproc.COLOR_BGR2HSV);
                Core.inRange(hsv, lowerHSV, upperHSV, mask);
                outputStream.putFrame(mask);
            }
             
            
            //System.out.println(new Date());
        }

    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());

    }
}
