package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;

public class Vision{

    private DoubleSubscriber blackRobotRelativeX;
    private DoubleSubscriber blackRobotRelativeY;
    private DoubleSubscriber blackRobotRelativeZ;

    private DoubleSubscriber blackClosestTag;
    private BooleanSubscriber blackTagDetected;

    private DoubleSubscriber blackFramerate;

    private DoubleSubscriber orangeRobotRelativeX;
    private DoubleSubscriber orangeRobotRelativeY;
    private DoubleSubscriber orangeRobotRelativeZ;

    private DoubleSubscriber orangeClosestTag;
    private BooleanSubscriber orangeTagDetected;

    private DoubleSubscriber orangeFramerate;

    private DoubleSupplier gyroAngle;

    public Vision(DoubleSupplier gyroAngle){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable blackVisionTable = inst.getTable("black_Fiducial");
        NetworkTable orangeVisionTable = inst.getTable("orange_Fiducial");

        blackRobotRelativeX = orangeVisionTable.getDoubleTopic("blackRelativeX").subscribe(0.0);
        blackRobotRelativeY = orangeVisionTable.getDoubleTopic("blackRelativeY").subscribe(0.0);
        blackRobotRelativeZ = orangeVisionTable.getDoubleTopic("blackRelativeZ").subscribe(0.0);

        blackClosestTag = blackVisionTable.getDoubleTopic("blackClosestTag").subscribe(0.0);
        blackTagDetected = blackVisionTable.getBooleanTopic("blackTagDetected").subscribe(false);

        blackFramerate = blackVisionTable.getDoubleTopic("blackFPS").subscribe(0.0);

        orangeRobotRelativeX = orangeVisionTable.getDoubleTopic("orangeRelativeX").subscribe(0.0);
        orangeRobotRelativeY = orangeVisionTable.getDoubleTopic("orangeRelativeY").subscribe(0.0);
        orangeRobotRelativeZ = orangeVisionTable.getDoubleTopic("orangeRelativeZ").subscribe(0.0);

        orangeClosestTag = orangeVisionTable.getDoubleTopic("orangeClosestTag").subscribe(0.0);
        orangeTagDetected = orangeVisionTable.getBooleanTopic("orangeTagDetected").subscribe(false);

        orangeFramerate = orangeVisionTable.getDoubleTopic("orangeFPS").subscribe(0.0);
    }
    
    public Pose2d relativeToGlobalPose2d(int tagID, Translation2d relativeCoords, Rotation2d gyroAngle){
        Pose2d tag2dPose = new Pose2d(VisionConstants.globalTagCoords[tagID][0],
            VisionConstants.globalTagCoords[tagID][1],
            new Rotation2d());

        Pose2d relative = new Pose2d(relativeCoords, gyroAngle);

        Transform2d relative2dTransformation = new Transform2d(relative.getTranslation(), relative.getRotation());

        Pose2d globalPose = tag2dPose.transformBy(relative2dTransformation.inverse());

        return new Pose2d(globalPose.getTranslation(), gyroAngle);
    }

    public Pose2d getBlackGlobalPose(){
        
        return relativeToGlobalPose2d(getBlackClosestTag(), 
            new Translation2d(getBlackRelativeX(), getBlackRelativeY()), 
            new Rotation2d(gyroAngle.getAsDouble()));
    }

    public double getBlackRelativeX(){
        return blackRobotRelativeX.get();
    }

    public double getBlackRelativeY(){
        return blackRobotRelativeY.get();
    }

    public double getBlackRelativeZ(){
        return blackRobotRelativeZ.get();
    }

    public int getBlackClosestTag(){
        return (int) blackClosestTag.get();
    }

    public double getBlackTimeStamp(){
        return blackRobotRelativeX.getLastChange();
    }

    public boolean getBlackTagDetected(){
        return blackTagDetected.get();
    }

    public double getBlackFPS(){
        return blackFramerate.get();
    }

    public Pose2d getOrangeGlobalPose(){
        
        return relativeToGlobalPose2d(getBlackClosestTag(), 
            new Translation2d(getBlackRelativeX(), getBlackRelativeY()), 
            new Rotation2d(gyroAngle.getAsDouble()));
    }
    
    public double getOrangeRelativeX(){
        return orangeRobotRelativeX.get();
    }

    public double getOrangeRelativeY(){
        return orangeRobotRelativeY.get();
    }

    public double getOrangeRelativeZ(){
        return orangeRobotRelativeZ.get();
    }

    public int getOrangeClosestTag(){
        return (int) orangeClosestTag.get();
    }

    public double getOrangeTimeStamp(){
        return orangeRobotRelativeX.getLastChange();
    }

    public boolean getOrangeTagDetected(){
        return orangeTagDetected.get();
    }

    public double getOrangeFPS(){
        return orangeFramerate.get();
    }
    
}
