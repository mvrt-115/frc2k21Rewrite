package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static final double kLimelightMountAngle = 15; //degrees
    public static final double kLimelightHeight = 45;  
    public static final double kTargetHeight = 98.25;


    public static enum LED_STATE {
        DEFAULT, ON, OFF, BLINKING;
    }

    public static enum CAM_MODE {
        VISION_WIDE, DRIVER, VISION_ZOOM;
    }

    private double ty;
    private double tx;
    private NetworkTable limelight;

    public Limelight() {
        ;
        
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        ty = limelight.getEntry("ty").getDouble(0.0);
        tx = limelight.getEntry("tx").getDouble(0.0);
    }

    public void setLED(LED_STATE newState) {
        switch (newState) {
        case ON:
            limelight.getEntry("ledMode").setNumber(3);
            break;
        case OFF:
            limelight.getEntry("ledMode").setNumber(1);
            break;
        case BLINKING:
            limelight.getEntry("ledMode").setNumber(2);
            break;
        case DEFAULT:
            limelight.getEntry("ledMode").setNumber(0);
            break;
        }
    }

    public void setPipeline(CAM_MODE newMode) {
        switch (newMode) {
        case VISION_WIDE:
            limelight.getEntry("pipeline").setNumber(0);
            break;
        case VISION_ZOOM:
            limelight.getEntry("pipeline").setNumber(1);
            break;
        case DRIVER:
            limelight.getEntry("pipeline").setNumber(2);
            break;
        }
    }

    /**
     * Get vertical angle
     * @return angle (degrees)
     */
    public double getTY() {
        if(targetsFound())
            ty = limelight.getEntry("ty").getDouble(0.0);
        return ty; 
    }

    /**
     * Get horizontal angle
     * @return angle (degrees)
     */
    public double getTX() {
        if(targetsFound())
            tx = limelight.getEntry("tx").getDouble(0.0);
        return tx;
    }

    /**
     * does inches
     * @return
     */
    public double getDistanceFromTarget() {
        getTY();
        double height = kTargetHeight - kLimelightHeight;
        double offsetAngle = Math.toRadians(kLimelightMountAngle + ty);
        double distance = height/Math.tan(offsetAngle);
        
        return distance;
    }

    /**
     * Whether limelight has found any valid targets
     * @return boolean
     */
    public boolean targetsFound() {
        int tv = (int)limelight.getEntry("tv").getDouble(0);
        if (tv == 1)
            return true;
        return false;
    }
}