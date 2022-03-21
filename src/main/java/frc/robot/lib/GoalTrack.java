// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * A class that is used to keep track of all goals detected by the vision
 * system. As goals are detected/not detected anymore by the vision system,
 * function calls will be made to create, destroy, or update a goal track.
 * 
 * This helps in the goal ranking process that determines which goal to fire
 * into, and helps to smooth measurements of the goal's location over time.
 * 
 * @see GoalTracker.java
 */
public class GoalTrack {
    Map<Double, Translation2d> mObservedPositions = new TreeMap<>();
    Translation2d mSmoothedPosition = new Translation2d();
    //int mId;

    public  GoalTrack(double timestamp, Translation2d first_observation) {
        mObservedPositions.put(timestamp, first_observation);
        mSmoothedPosition = first_observation;
    }

    public void reset(){
        mObservedPositions=new TreeMap<>();
        mSmoothedPosition=new Translation2d();
    }

    /**
     * Makes a new track based on the timestamp and the goal's coordinates (from
     * vision)
     */
    /*public static GoalTrack makeNewTrack(double timestamp, Translation2d first_observation) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        
        return rv;
    }*/

    public void emptyUpdate() {
        pruneByTime();
    }

    /**
     * Attempts to update the track with a new observation.
     * 
     * @return True if the track was updated
     */
    public boolean tryUpdate(double timestamp, Translation2d new_observation) {
        //if (!isAlive()) {
       //     return false;
       // }
      /* if(new_observation==null) {
           //System.out.println("new ob null");
           return false;
       }
       if(mSmoothedPosition==null) {
           //System.out.println("smooth null");
           return false;
       }*/
      //  double distance = mSmoothedPosition.inverse().translateBy(new_observation).norm(); //I believe this is distance apart from previous targets (won't count if not close enough)
     //   if (distance < Constants.kMaxTrackerDistance) {
            mObservedPositions.put(timestamp, new_observation);
            pruneByTime();
           // System.out.println(mObservedPositions);
            return true;
     //   } else {
     //       emptyUpdate();
           // return false;
      //  }
    }

    public boolean hasData() {
       return mObservedPositions.size() > 0;
    }

    /**
     * Removes the track if it is older than the set "age" described in the
     * Constants file.
     * 
     * @see Constants.java
     */
    void pruneByTime() { //called each time there is an update
        double delete_before = Timer.getFPGATimestamp() - Constants.kMaxGoalTrackAge;
        for (Iterator<Map.Entry<Double, Translation2d>> it = mObservedPositions.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Translation2d> entry = it.next();
            if (entry.getKey() < delete_before) {
                it.remove();
            }
        }
       // if (mObservedPositions.isEmpty()) { //if all of them are too old, removes it
      //      mSmoothedPosition = new Translation2d();
      //  } else {
            smooth();
      //  }
    }

    /**
     * Averages out the observed positions based on an set of observed positions
     */
    void smooth() {
        if (hasData()) {
            double x = 0;
            double y = 0;
            for (Map.Entry<Double, Translation2d> entry : mObservedPositions.entrySet()) {
                x += entry.getValue().getX();
                y += entry.getValue().getY();
            
            }
            x /= mObservedPositions.size();
            y /= mObservedPositions.size();
            mSmoothedPosition = new Translation2d(x, y);
        }
    }

    public Translation2d getSmoothedPosition() {
        //System.out.println("(" + mSmoothedPosition.getX() + ", " + mSmoothedPosition.getY() + ")\n");
        return mSmoothedPosition;
    }

    public double getLatestTimestamp() {
       return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

//   public double getStability() {
 //       return Math.min(1.0, mObservedPositions.size() / (Constants.kCameraFrameRate * Constants.kMaxGoalTrackAge));
 //   }

  /*  public int getId() {
        return mId;
    }*/
}
