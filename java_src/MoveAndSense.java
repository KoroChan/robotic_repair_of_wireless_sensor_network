

import java.io.IOException;
import java.util.ArrayList;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.NavigationListener;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.robotics.pathfinding.Path;

public class MoveAndSense implements NavigationListener {
    
    // DEMO ONLY
    private Controller ctrlr;
    
    public void setController( Controller ctrlr ) {
        this.ctrlr = ctrlr;
    }
    // *********
    
    // Diameter of the wheels (cm).
    private static final double WHEEL_D = 4.32;
    
    // Distance between the centres of the wheels (cm).
    private static final double TRACK_W = 11.7;
    
    private final Navigator navigator;
    
    private final RangeFeatureDetector sensor;
    
    private boolean initialised;
    
    private boolean stopped;
    
    private final ArrayList<PlanListener> listeners;

    /**
     * @param sensor The long-range sensor to use.
     */
    public MoveAndSense( RangeFeatureDetector sensor ) {
        DifferentialPilot pilot;
        pilot = new DifferentialPilot( WHEEL_D, TRACK_W, Motor.A, Motor.C );
        /*
         * TODO: Assuming the robot starts at coordinates (0, 0).
         * How to provide alternative starting coordinates?
         */
        OdometryPoseProvider odom = new OdometryPoseProvider( pilot );
        navigator = new Navigator( pilot, odom );
        this.sensor = sensor;
        initialised = false;
        stopped = true;
        listeners = new ArrayList<>();
    }
    
    /**
     * Register a listener to be notified of events related to the execution of
     * plans.
     * @param listener 
     * @throws IllegalArgumentException if {@code listener} is {@code null}.
     */
    public void addPlanListener( PlanListener listener ) {
        if ( listener != null ) {
            if ( ! listeners.contains( listener ) ) {
                listeners.add( listener );
            }
        } else {
            throw new IllegalArgumentException();
        }
    }
    
    /**
     * Do not notify the given object of future events.
     * @param listener
     * @return {@code true} if {@code listener} was previously registered.
     */
    public boolean removePlanListener( PlanListener listener ) {
        return listeners.remove( listener );
    }
    
    /**
     * Set this module to an appropriate state to begin executing plans.
     * @return {@code true} the first time this method is called on this object.
     */
    public boolean init() {
        if ( ! initialised ) {
            // Cannot reference this in the constructor.
            navigator.addNavigationListener( this );
            return initialised = true;
        } else {
            return false;
        }
    }
    
    /**
     * Start the robot following the given plan, scanning for obstacles
     * while traveling.
     * @param plan
     * @throws IllegalStateException if {@link #init()} has not been called.
     */
    public void execute( Path plan ) {
        if ( initialised ) {
            stopped = false;
            navigator.setPath( plan );
            /*
             * Begin following the plan, stopping at each way-point to look
             * for obstacles. Can turn off sensor when moving between way-points
             * because the robot will only move if it has not detected an
             * obstacle to the next way-point.
             */
            sensor.enableDetection( false );
            navigator.singleStep( true );
            navigator.followPath();
        } else {
            throw new IllegalStateException();
        }
    }
    
    /**
     * The robot will finish traveling to the current way-point in its plan if
     * it has not yet reached it, but will not travel to the 
     * following way-point.
     */
    public void stop() {
        stopped = true;
    }
    
    /**
     * @return The current coordinates and heading of the robot.
     */
    public Pose getPose() {
        return navigator.getPoseProvider().getPose();
    }
    
    /**
     * @return {@code true} if the robot is moving towards a way-point in its
     * plan.
     */
    public boolean isMoving() {
        return navigator.isMoving();
    }

    @Override
    public void atWaypoint( Waypoint waypoint, Pose pose, int sequence ) {
        Waypoint nextWaypoint = navigator.getWaypoint();
        String message = "robot at ( " +
                         (int) pose.getX() + ", " + (int) pose.getY() + " )";
        try {
            ctrlr.dos.writeBytes( message );
            ctrlr.dos.flush();
        } catch (IOException ex) {
            System.out.println( "IO error" );
        }
        
        if ( nextWaypoint != null ) {
            // Another waypoint follows.
            float relativeBearing = pose.relativeBearing( nextWaypoint );
            // Turn the robot to face the next waypoint.
            navigator.rotateTo( pose.getHeading() + relativeBearing );
            // Scan for an obstacle that will block the path.
            sensor.enableDetection( true );
            // Give the sensor a chance to work.
            try {
                Thread.sleep( 2 * Controller.SENS_P );
            } catch ( InterruptedException ex ) {
                /*
                 * Might have been interrupted before the sensor has had a
                 * chance to work. If this is the case, stop() might be called
                 * too late. The code that calls stop() should check if the
                 * robot has already started moving again.
                 */
            }
            if ( ! stopped ) {
                // Continue on the current path.
                sensor.enableDetection( false );
                navigator.followPath();
            }
        }
    }

    @Override
    public void pathComplete( Waypoint waypoint, Pose pose, int sequence ) {
        for ( PlanListener listener : listeners ) {
            listener.planExecuted();
        }
    }

    @Override
    public void pathInterrupted(Waypoint waypoint, Pose pose, int sequence) {
        // TODO
        LCD.clear();
        LCD.drawString( "Path interrupt", 0, 0 );
    }
    
}
