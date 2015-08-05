import lejos.geom.Line;
import lejos.geom.Rectangle;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.pathfinding.FourWayGridMesh;

public class FourWayGridMeshFactory {
    
    public static FourWayGridMesh squareGridMesh( int gridSize, 
                                                  float gridSquareSide ) {
        float boundingBoxSide = ( 1 + gridSize ) * gridSquareSide;
        Rectangle boundingBox = new Rectangle( 0.0f, 0.0f, boundingBoxSide, boundingBoxSide );
        LineMap mapGeometry = new LineMap( new Line[0], boundingBox );
        
        float clearance = gridSquareSide / 2;
        FourWayGridMesh grid = new FourWayGridMesh( mapGeometry, gridSquareSide,
                                                    clearance );
        
        grid.regenerate();
        return grid;
    }
}