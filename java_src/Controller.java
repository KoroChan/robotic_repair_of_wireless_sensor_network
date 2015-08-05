import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import lejos.geom.Point;
import lejos.geom.Rectangle;
import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.USB;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Pose;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.robotics.pathfinding.AstarSearchAlgorithm;
import lejos.robotics.pathfinding.FourWayGridMesh;
import lejos.robotics.pathfinding.Node;
import lejos.robotics.pathfinding.Path;


public class Controller implements FeatureListener, PlanListener {
    
    // DEMO ONLY
    public NXTConnection connection;
    public DataInputStream dis;
    public DataOutputStream dos;
    // *********
    
    /**
     * Maximum distance at which the robot will sense for obstacles (cm).
     */
    public static final float MAX_SENS_R = 34.0f;
    
    /**
     * The number of milliseconds between pulses by the ultrasonic sensor.
     */
    public static final int SENS_P = 250;
    
    /**
     * The distance from the front of the ultrasonic sensor to the center of the
     * robot (cm).
     */
    public static final float SENSOR_OFFSET = 9.0f;
    
    /**
     *
     */
    public static final float ROBOT_LENGTH = 26.0f;
    
    private static final String NO_PATH = "Couldn't find a path to the target.";
    
    private static final String LOCALISE_FAIL = "Localisation failure.";
    
    private static final String MAPPING_FAIL = "Mapping failure.";
    
    private FourWayGridMesh map;
    
    private Node target;
    
    private Node start;
    
    // TODO: Move path finding functionality to from NXT to Galileo.
    private AstarSearchAlgorithm pathFinder;
    
    private MoveAndSense moveAndSense;
    
    private boolean initialised;
    
    public Controller() {
        connection = USB.waitForConnection( 0, NXTConnection.PACKET );
        dis = connection.openDataInputStream();
        dos = connection.openDataOutputStream();
        
        pathFinder = new AstarSearchAlgorithm();
        initialised = false;
    }
    
    public boolean init() {
        if ( ! initialised ) {            
            // Initialise long range sensor.
            UltrasonicSensor sonar = new UltrasonicSensor( SensorPort.S4 );
            RangeFeatureDetector sensor;
            sensor = new RangeFeatureDetector( sonar, MAX_SENS_R, SENS_P );
            /*
             * Do not want to be notified of obstacles yet because the robot
             * hasn't started moving.
             */
            sensor.enableDetection( false );
            sensor.addListener( this );
            
            moveAndSense = new MoveAndSense( sensor );
            moveAndSense.init();
            moveAndSense.addPlanListener( this );
            // ********************************
            moveAndSense.setController( this );
            // ********************************
            return initialised = true;
        } else {
            return false;
        }
    }
    
    public void run( FourWayGridMesh map, Node start, Node target ) 
            throws DestinationUnreachableException {
        if ( initialised ) {
            this.map = map;
            Collection<Node> locations = map.getMesh();
            if ( locations.contains( start ) && locations.contains( target ) ) {
                // TODO: Move path finding functionality to from NXT to Galileo.
                this.start = start;
                this.target = target;
                Path plan = pathFinder.findPath( start, target );
                if ( plan != null ) {
                    // Begin executing plan.
                    moveAndSense.execute( plan );
                } else {
                    throw new DestinationUnreachableException();
                }
            }
        } else {
            throw new IllegalStateException();
        }
    }
    

    @Override
    public void featureDetected( Feature feature, FeatureDetector detector ) {
        moveAndSense.stop();
        // The robot is no longer moving, so sensor can be turned off.
        detector.enableDetection( false );
        Pose pose = moveAndSense.getPose();
        float realRange = feature.getRangeReading().getRange() + SENSOR_OFFSET;
        Point detected = pose.pointAt( realRange, pose.getHeading() );
        
        Node robotLoc = localiseRobot( pose );
        
        if ( robotLoc != null ) {
            Node obstacleLoc = mapObstacle( robotLoc, detected );
            if ( obstacleLoc != null ) {
                // *************************************************
                String message = "Feature at ( " +
                                 (int) obstacleLoc.x + ", " + 
                                 (int) obstacleLoc.y + " )";
                try {
                    dos.writeBytes( message );
                    dos.flush();
                } catch (IOException ex) {
                    System.out.println( "IO error" );
                }
                // *************************************************
                
                map.removeNode( obstacleLoc );
                // TODO: Move path finding functionality to from NXT to Galileo.
                Path newPlan = pathFinder.findPath( robotLoc, target );
                if ( newPlan != null ) {
                    moveAndSense.execute( newPlan );
                } else {
                    System.out.println( NO_PATH );
                }
            } else {
                System.out.println( MAPPING_FAIL );
            }
        } else {
            System.out.println( LOCALISE_FAIL );
        }
    }

    @Override
    public void planExecuted() {
        try {
            // Follow the plan in reverse.
            run( map, target, start );
        } catch ( DestinationUnreachableException ex ) {
            System.out.println( NO_PATH );
        }
    }
    
    private Node localiseRobot( Pose pose ) {
        double robotLocErr = Double.MAX_VALUE;
        Node robotLoc = null;
        for ( Node location : map.getMesh() ) {
            Point locPoint = new Point( location.x, location.y );
            double rd = pose.distanceTo( locPoint );
            if ( rd < robotLocErr ) {
                robotLocErr = rd;
                robotLoc = location;
            }
        }
        return robotLoc;
    }
    
    private Node mapObstacle( Node robotLoc, Point detected ) {
        Node obstacleLoc = null;
        for ( Node neighbour : robotLoc.getNeighbors() ) {
            /*
             * The idea is to see if the point on the obstacle that was
             * detected falls within any of the grid squares that neighbour
             * the square containing the robot.
             */
            Rectangle gridSquare;
            /*
             * TODO: 17.0 here is half the side length of grid squares.
             * We use it to get the coordinates of the top-left corner of
             * each grid square neighbouring the square containing the robot.
             * Where to store this value so the magic constant is not used?
             */
            float xCoord = neighbour.x - 17.0f;
            float yCoord = neighbour.y - 17.0f;
            /*
             * TODO: 34.0 here is the side length of the grid squares.
             * Where to store this value so the magic constant is not used?
             */
            gridSquare = new Rectangle( xCoord, yCoord, 34.0f, 34.0f );
            if ( gridSquare.contains( detected ) ) {
                obstacleLoc = neighbour;
                break;
            }
        }
        return obstacleLoc;
    }
    
    public static void main( String[] args ) {
        /*
         * We want the centre of the robot (wheel axis) to align with the 
         * grid points, which are the centre points of squares.
         */
        float gridSquareSide = 34.0f; //( ROBOT_LENGTH - SENSOR_OFFSET ) * 2;
        FourWayGridMesh map;
        map = FourWayGridMeshFactory.squareGridMesh( 3, gridSquareSide );
        ArrayList<Node> locations = new ArrayList<>( map.getMesh() );
        
        Controller ctrlr = new Controller();  
        try {
            byte[] endPoints = new byte[2];
            int messageLen = 0;
            while ( ( messageLen = ctrlr.dis.available() ) == 0 );
            ctrlr.dis.read( endPoints, 0, endPoints.length );
            Node start = locations.get( endPoints[0] );
            Node target = locations.get( endPoints[1] );
            
            ctrlr.init();
            ctrlr.run( map, start, target );
        } catch ( DestinationUnreachableException ex ) {
            System.out.println( NO_PATH );
        } catch (IOException ex) {
            System.out.println( "IO error" );
            ctrlr.connection.close();
        }
        Button.waitForAnyPress();
        ctrlr.connection.close();
    }
}
