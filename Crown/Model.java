import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.lang.Math;

import com.google.gson.JsonObject;

import heronarts.lx.LX;
import heronarts.lx.LXLayer;
import heronarts.lx.LXLoopTask;
import heronarts.lx.model.LXAbstractFixture;
import heronarts.lx.model.LXModel;
import heronarts.lx.model.LXPoint;
import heronarts.lx.transform.LXTransform;

import toxi.geom.Vec2D;
import toxi.geom.Vec3D;
import toxi.geom.Triangle3D;

// for the STL parser
import hall.collin.christopher.stl4j.STLParser;
import hall.collin.christopher.stl4j.Triangle;
import hall.collin.christopher.stl4j.Vec3d;
import java.io.IOException;
import java.nio.file.Paths;
import java.nio.file.Path;


class Geometry {

  // we are in MM in this model
  // change to inches to see how the patterns react
  final static float INCHES = 1.0f;
  final static float INCH = INCHES;
  final static float FEET = 12 * INCHES;
  final static float FOOT = FEET;
  
  final static float MMS_INCHES = 25.4f;
  

  /**
   * This defines the positions of the towers, which are
   * x (left to right), z (front to back), and rotation
   * in degrees.
   */
  final static float[][] TOWER_POSITIONS = {
    /*  X-pos    Z-pos    Rot  */
    {  9*Geometry.FEET,  -9*Geometry.FEET,  (float) ( - Math.PI / 4.0f ) },
    {  9*Geometry.FEET,  9*Geometry.FEET,   (float) (Math.PI / 4.0f)  },
    {  -9*Geometry.FEET,  9*Geometry.FEET, (float) ( Math.PI / 0.75f)  },
    {  -9*Geometry.FEET,  -9*Geometry.FEET, (float) ( - Math.PI / 0.75f)  }
  };

  final static float FENCE_LED_INSET = 4 * INCHES;
  
  public final float FENCE_TRANSFORM[][] = {{12*FEET, 0.0f, -12*FEET}, {12*FEET , 0.0f, 12*FEET}, 
                        {-12*FEET, 0.0f, 12*FEET}, {-12*FEET, 0.0f, -12*FEET} };  

  // Tower information
  
  // The shape is the radial distance of each bulb in the string, going upward.
  
  final static float towerBulbShape[][] = {
      { 4.0f, 6.0f, 8.0f, 12.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 20.0f, 16.0f },
      { 14.0f, 20.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 14.0f },
      { 14.0f, 24.0f, 22.0f, 20.0f, 18.0f, 16.0f, 14.0f, 12.0f, 10.0f, 8.0f, 6.0f, 4.0f }
  };
  
  final static float towerInitialY = 5.5f * FEET; 
  final static float towerBulbDistY = 8.0f * INCH;
  final static float towerSegmentDistY = 11.0f * INCH;
  
  public final static float HEIGHT = 720;
  
  /*
  ** TOdo: insert more components for the real three jointed towers
  */

  Geometry() {
    //distances = new float[(int) (HEIGHT/BEAM_SPACING + 2)];
    //heights = new float[(int) (HEIGHT/BEAM_SPACING + 2)];
    //for (int i = 0; i < heights.length; ++i) {
    //  heights[i] = Utils.min(HEIGHT, i * BEAM_SPACING);
    //  distances[i] = distanceFromCenter(heights[i]);
    //}
  }
  
  float distanceFromCenter(float atHeight) {
    //float oppositeLeg = VERTICAL_MIDPOINT - atHeight;
    //float angle = Utils.asin(oppositeLeg / MIDDLE_RADIUS);
    //float adjacentLeg = MIDDLE_RADIUS * Utils.cos(angle);
    //return MIDDLE_DISTANCE - adjacentLeg;  
    return(0.0f);
  }
  
  float angleFromAxis(float atHeight) {
    // This is some shitty trig. I am sure there
    // is a simpler way but it wasn't occuring to me.
    //float x1 = MIDDLE_DISTANCE - distanceFromCenter(atHeight);
    //float a1 = Utils.acos(x1 / MIDDLE_RADIUS); 
    
    //float r = MIDDLE_RADIUS;
    //float y = Cluster.BRACE_LENGTH / 2;
    //float a = Utils.asin(y/r);
    //float a2 = a1 - 2*a;
    
    //float x2 = Utils.cos(a2) * MIDDLE_RADIUS;
    
    //return Utils.asin((x2-x1) /Cluster.BRACE_LENGTH); 
    return(0.0f);
  }
}


class Model extends LXModel {
  
  /**
   * Towers in the model - there is 4
   */
  public final List<Tower> towers;
  
  /*
  ** The Fence
  */
  public final Fence fence;

  
  /**
   * Clusters in the model - the fence and the towers each have clusters
   * A cluster is a set of Bulbs inside the model
   */
  public final List<Cluster> clusters;
  
  /**
   * Lookup table from cluster UID to cluster object.
   */
  public final Map<String, Cluster> clustersByIp;
  
  /**
   * Total Bulbs
   */
  public final List<Bulb> bulbs;

  public final static Geometry geometry = new Geometry();

  private final ArrayList<ModelTransform> modelTransforms = new ArrayList<ModelTransform>();
  
  final String dataPath; // this is the filepath from LX

    
  Model(List<ClusterConfig> clusterConfig, String dataPath) {
      
    super(new Fixture(geometry, clusterConfig, dataPath));
    this.dataPath = dataPath;
    
    Fixture f = (Fixture) this.fixtures.get(0);
    
    this.towers = Collections.unmodifiableList(f.towers);
    this.fence = f.fence;
    
    // Set up the clusters and clustersByIp structs
    List<Cluster> _clusters = new ArrayList<Cluster>();
    Map<String, Cluster> _clustersByIp = new HashMap<String, Cluster>();

    for (Tower tower : this.towers) {
        _clusters.add(tower);
        _clustersByIp.put(tower.getIpAddress(), tower);
    }
    
    _clusters.add(this.fence);
    _clustersByIp.put(this.fence.getIpAddress(), this.fence);
    
    this.clusters = Collections.unmodifiableList(_clusters);
    this.clustersByIp = Collections.unmodifiableMap(_clustersByIp);
    
    // Now all the bulbs, since you have a cluster list
    List<Bulb> _bulbs = new ArrayList<>();
    for (Cluster cluster : this.clusters) {
      for (Bulb bulb : cluster.getBulbs()) {
        _bulbs.add(bulb);
      }
    }
    this.bulbs = Collections.unmodifiableList(_bulbs);

  }

  
  private static class Fixture extends LXAbstractFixture {
    
    final List<Tower> towers = new ArrayList<Tower>();
    
    final Fence fence;
    
    private Fixture(Geometry geometry, List<ClusterConfig> clusterConfig, String dataPath) {
        
      int towerIndex = 0;
      for (float[] towerPosition : Geometry.TOWER_POSITIONS) {
        towers.add(new Tower(geometry, clusterConfig, towerIndex++, towerPosition[0], towerPosition[1], towerPosition[2]));
      }
      for (Tower tower : towers) {
        for (Bulb bulb : tower.bulbs) {
           points.add(bulb);
        }
      }
      
      fence = new Fence(geometry, clusterConfig, dataPath );
      for (Bulb bulb: fence.bulbs) {
          points.add(bulb);
      }
      
    }
  }

  public void addModelTransform(ModelTransform modelTransform) {
    modelTransforms.add(modelTransform);
  }

  public void runTransforms() {
    for (Bulb bulb : bulbs) {
      bulb.resetTransform();
    }
    for (ModelTransform modelTransform : modelTransforms) {
      if (modelTransform.isEnabled()) {
        modelTransform.transform(this);
      }
    }
    for (Bulb bulb : bulbs) {
      bulb.didTransform();
    }
  }
}


// In "better java" there would probably be a Model class,
// with subtypes Fence and Tower... I've used an interface for cluster and jammed
// some things in there

class ClusterConfig {
  String type;
  int towerIndex;
  String ipAddress;
  float x;
  float y;
  float z;
}


class Fence extends LXModel implements Cluster  {
  
  /**
   * Bulbs in the fence
   */
  public final List<Bulb> bulbs;
  
  public String ipAddress;
  
  /*
  **
  */
  
  public final int id;
  
  /**
   * x-position of center - ie, 0
   */
  public final float x;
  
  /**
   * z-position of center of base of tower
   */
  public final float z;
  
  /**
   * Rotation in degrees of tower about vertical y-axis
   */
  public final float ry;
    
  public List<Bulb> getBulbs() { return ( bulbs ); }
  
  public String getIpAddress() { return ( ipAddress ); }
  
  List<Triangle3D> fenceSTL;
  
  /*
  **
  */
  
  Fence( Geometry geometry, List<ClusterConfig> clusterConfig, String dataPath ) {
      
    super(new Fixture(geometry, clusterConfig, 0.0f, 0.0f, 0.0f));
    
    Fixture f = (Fixture)this.fixtures.get(0);
    this.bulbs = f.bulbs;
    this.ipAddress = f.ipAddress; 
    
    this.x = 0.0f;
    this.z = 0.0f;
    this.ry = 0.0f;
    
    this.id = 0; // happens to be only one
    
    List<Triangle> rawSTL = null;
    
    // Load the fence triangles - TODO: get the file path right
    try {
        rawSTL = STLParser.parseSTLFile(Paths.get(dataPath +"fence.stl") );
    } catch (IOException ex) {
        System.out.println( ex.toString() );
        System.out.println("Could not find fence STL file");
    }
    System.out.println("Loaded fence Ntriangles "+ ( rawSTL != null ? rawSTL.size() : 0 ) );
    
    // Find out the size. Then we will zero the coordinates.
    double maxX= - Double.MAX_VALUE, minX= Double.MAX_VALUE;
    double maxY= - Double.MAX_VALUE, minY= Double.MAX_VALUE;
    double maxZ= - Double.MAX_VALUE, minZ= Double.MAX_VALUE;
    for (Triangle t : rawSTL ) {
        Vec3d v[] = t.getVertices();
        for (int i=0;i<3;i++) {
            if (maxX < v[i].x) maxX = v[i].x;
            if (maxY < v[i].y) maxY = v[i].y;
            if (maxZ < v[i].z) maxZ = v[i].z;
            
            if (minX > v[i].x) minX = v[i].x;
            if (minY > v[i].y) minY = v[i].y;
            if (minZ > v[i].z) minZ = v[i].z;
        }
    }
    //System.out.printf("Size of fence object: x: %f %f , y: %f %f, z: %f %f\n",
    //    minX, maxX, minY, maxY, minZ, maxZ );
    
    // Copy into the Toxi structures someone liked at some time
    // CONVERT MM TO INCHES
    fenceSTL = new ArrayList<>();
    
    float xDelta = - (float)maxX;
    float yDelta = - (float)minY;
    float zDelta = - (float)minZ;
    float scale = 1.0f / geometry.MMS_INCHES;
    for (Triangle t : rawSTL ) {
        Vec3d v[] = t.getVertices();
        Vec3D a = new Vec3D( ((float)v[0].x + xDelta) * scale, 
                             ((float)v[0].y + yDelta) * scale, 
                             ((float)v[0].z + zDelta) * scale );
        Vec3D b = new Vec3D(  ((float)v[1].x + xDelta) * scale,  
                               ((float)v[1].y + yDelta) * scale,
                               ((float) v[1].z + zDelta) * scale );
        Vec3D c = new Vec3D( ((float)v[2].x + xDelta) * scale,  
                             ((float)v[2].y + yDelta) * scale, 
                             ((float) v[2].z + zDelta) * scale );
        fenceSTL.add ( new Triangle3D( a, b, c ) );
    }
    
  }
  
  // FENCE
  private static class Fixture extends LXAbstractFixture {
    
    final List<Bulb> bulbs;
    
    final String ipAddress;
    
    Fixture(Geometry geometry, List<ClusterConfig> clusterConfig, float x, float z, float ry) {
        
      Vec3D fenceCenter = new Vec3D(0, 0, 0);
      
      this.bulbs = new ArrayList<>();
      
      String _ipAddress = null;
      for (ClusterConfig cp : clusterConfig) {
        if (cp.type.equals("fence") ) {
          _ipAddress = cp.ipAddress;
        }
      }
      if (_ipAddress == null) { System.out.println("must have a fence in the config"); }
      this.ipAddress = _ipAddress;
      
      // The centers of each bulb - vertically within the wall - 6 strands
      // Eric told me 8 inches, with the gap, but those look wrong.
      //float centerHeights[] = { 60*Geometry.INCH, 52*Geometry.INCH, 35*Geometry.INCH, 27*Geometry.INCH, 19*Geometry.INCH , 11*Geometry.INCH };
      float centerHeights[] = { 50*Geometry.INCH, 42*Geometry.INCH, 35*Geometry.INCH, 27*Geometry.INCH, 19*Geometry.INCH , 11*Geometry.INCH };
      int panelsPerSide = 6;
      int bulbsPerPanel = 5;
      int bulbsPerSide = ( panelsPerSide * bulbsPerPanel );
      float fenceWidth = 22.5f *Geometry.FEET; // The entire width of the box
      float bulbDistance = fenceWidth / ( panelsPerSide * bulbsPerPanel );
      // NDB starts in some corner --- let's say South East
      
      // Add all the Bulbs to the Cluster
      // Add in the same order the NDB will be addressing them,
      // which means row by row fgoing down
      int clusterIndex = 0;
      for ( int i = 0 ; i < centerHeights.length; i++ ) {
          // Eastern edge
          for ( int j = 0; j < bulbsPerSide ; j++ ) {
              // TODO: The "transform" here is an LX transform and it's not clear how it's attached to the LXpoint
              LXTransform t = new LXTransform();
              float bx = (fenceWidth / 2) - Geometry.FENCE_LED_INSET;
              float bz = (- fenceWidth / 2) + (j * bulbDistance );
              Bulb b = new Bulb( clusterIndex, "fence", 0, fenceCenter, t, bx, centerHeights[i], bz, 0.0f, Utils.PI/2, 0.0f );
              bulbs.add(b);
              clusterIndex++;
          }
          // North edge
          for ( int j = 0; j < bulbsPerSide ; j++ ) {
              LXTransform t = new LXTransform();
              float bx = (fenceWidth / 2) - ( j * bulbDistance );
              float bz = (fenceWidth / 2) - Geometry.FENCE_LED_INSET;
              Bulb b = new Bulb( clusterIndex, "fence", 0, fenceCenter, t, bx, centerHeights[i], bz, 0.0f, 0.0f, 0.0f);
              bulbs.add(b);
              clusterIndex++;
          }
          // South Edge
          for ( int j = 0; j < bulbsPerSide ; j++ ) {
              LXTransform t = new LXTransform();
              float bx = (fenceWidth / 2) - ( j * bulbDistance );
              float bz = (- fenceWidth / 2) + Geometry.FENCE_LED_INSET;
              Bulb b = new Bulb( clusterIndex, "fence", 0, fenceCenter, t, bx, centerHeights[i], bz, 0.0f, 0.0f, 0.0f );
              bulbs.add(b);
              clusterIndex++;
          }
          // West edge
          for ( int j = 0; j < bulbsPerSide ; j++ ) {
              LXTransform t = new LXTransform();
              float bx = (- fenceWidth / 2) + Geometry.FENCE_LED_INSET;
              float bz =  (- fenceWidth / 2) + ( j * bulbDistance );
              Bulb b = new Bulb( clusterIndex, "fence", 0, fenceCenter, t, bx, centerHeights[i], bz, 0.0f, Utils.PI/2, 0.0f );
              bulbs.add(b);
              clusterIndex++;
          }
      }
    }
  }
}


class Tower extends LXModel implements Cluster  {
  
  /**
   * Bulbs in a tower
   */
  public final List<Bulb> bulbs;
  
  /**
   * index of the tower
   */
  public final int index;
  
  /**
   * x-position of center of base of tower
   */
  public final float x;
  
  /**
   * z-position of center of base of tower
   */
  public final float z;

  /**
   * Rotation in degrees of tower about vertical y-axis
   */
  public final float ry; 

  public final String ipAddress;

  public List<Bulb> getBulbs() { return ( bulbs ); }

  
  // Since the shape of each segment is non-linear, just have a hardcoded
  // array of distances from the radius. The three segments are the lower (0), middle(1), and upper (2)
  // to be more correct, there needs to be a little trig, because the wires between bulbs have 8 inches of slack,
  // so you can't have 8 inches, plus some away-from-center, but this is just a model for visualization
  //
  // Each "string" has 12 "bulbs". There are, radially, 12 strings per segment, Each bulb has 8 inches aparet.
  //
  // There is a difference in geometry that the top one, in particular, is not the same radially, but 
  // doing something simpler for now
  // We will make these inches for now...
  // Since there are 12 componets 8inches wide, the minimum radius is around 48 where they don't overlap... obviously
  // some overlap.... but let's get around 48" radius
  //
  // We might have to calculate an individual "bulb width" based on these radiuses.... otherwise, ???
  //, like a 4 inch radius has very small bulbs
  


  public String getIpAddress() { return ( ipAddress ); }

  Tower(Geometry geometry, List<ClusterConfig> clusterConfig, int towerIndex, float x, float z, float ry) { 
    super(new Fixture(geometry, clusterConfig, towerIndex, x, z, ry));
    Fixture f = (Fixture)this.fixtures.get(0);
    this.index = towerIndex;
    this.bulbs = f.bulbs;
    this.ipAddress = f.ipAddress; 
    this.x = x; 
    this.z = z; 
    this.ry = ry; 
  }  
  
  //Tower
  private static class Fixture extends LXAbstractFixture { 
       
    final List<Bulb> bulbs; 
    
    final String ipAddress; 
    
    Fixture(Geometry geometry, List<ClusterConfig> clusterConfig, int towerIndex, float towerX, float towerZ, float towerRY) {

      Vec3D towerCenter = new Vec3D(towerX, 0, towerZ);
      LXTransform t = new LXTransform();
      t.translate(towerX, 0, towerZ);
      t.rotateY(towerRY * Utils.PI / 180);
      
      String _ipAddress = null;
      bulbs = new ArrayList<>();
      
      for (ClusterConfig cp : clusterConfig) {
        if (cp.type.equals("tower") && cp.towerIndex == towerIndex) {
          _ipAddress = cp.ipAddress;
         }
      }
      this.ipAddress = _ipAddress;
      
      int clusterIndex = 0;
      
      // Insert the bulbs
      // 12 strings, each one at the same radial position, stepping around

      float rad = 0.0f;
      for (int radialStep = 0; radialStep < 12; radialStep++) {
          float _y = geometry.towerInitialY;
          rad += ( 2.0f * (float)Math.PI ) / 12.0f;
      
          for (int seg = 0; seg < 3; seg++) {
              for (float radialDistance : geometry.towerBulbShape[seg]) {
      
                  float _x = (float)Math.cos(rad) * (radialDistance * geometry.INCH);
                  float _z = (float)Math.sin(rad) * (radialDistance * geometry.INCH);
                  
                  Bulb b = new Bulb( clusterIndex, "tower", towerIndex, towerCenter, t, _x, _y, _z, 0.0f,((float)Math.PI / 2) + rad, 0.0f);
                  bulbs.add(b);
                  clusterIndex++;
              
                  _y += geometry.towerBulbDistY;
                  
              }
              _y += geometry.towerSegmentDistY;
          } 

      }


    }
  }
}

class Bulb extends LXPoint {

  // Might have to change this if I have multiple bulb sizes
  public static final float BULB_HEIGHT = 7*Geometry.INCHES;
  public static final float BULB_WIDTH = 7*Geometry.INCHES;
  public static final float BULB_DEPTH = 2*Geometry.INCHES;

  
  /**
   * Index of this bulb in cluster, in DMX order, for sending packets
   */
  public final int clusterPosition;
  
  /**
   * Pitch of bulb, in degrees, relative to model
   */
  public final float rx;
  
  /**
   * Yaw of bulb, in degrees, relative to model, after pitch
   */
  public final float ry;
  
  /**
   * Roll of bulb, in degrees, relative to model, after pitch+yaw
   */
  public final float rz;
  
  /**
   * x-position of bulb, relative to 0 of tower 
   */
  public final float tx;
  
  /**
   * y-position of bulb, relative to 0 of tower 
   */
  public final float ty;
  
  /**
   * z-position of bulb, relative to 0 of tower 
   */
  public final float tz;
  
  /**
   * Radial distance from bulb center to center of tower ( ie relative ) in x-z plane 
   */
  public final float r;
  
  /**
   * Angle in degrees from bulb center to center of tower ( ie relative ) in x-z plane
   */
  public final float theta;
  
  /**
   * Point of the bulb in the form (theta, y) relative to center of tower base
   */
  public final Vec2D cylinderPoint;
  
  /*
  ** in the case of Crown, we may want Patterns to know about the
  ** bulb's type. Call me simple, but it just doesn't
  ** seem easy to figure out how to pass a more generic type ( we could pass LXModel )
  ** and we could pass a MethodReference, but I don't see that buying anything....
  ** we don't need this to be dynamic ( bulbs don't move around )
  ** so... how about just constructing with the model type and ID.
  */
  
  public final String modelType;
  public final Vec3D modelCenter;
  public final int modelID;
  
  
  /* NB. x,y,z here are relative to the model center
  **
  ** People might also want "bulb type", to only effect the fence or a particular tower,
  ** without looking at the actual X,Y coords.
  */

  public float transformedY;
  public float transformedTheta;
  public Vec2D transformedCylinderPoint;

  // x,y,z are relative to the MODEL.
  // The 'cluster' is known in order to determine the model type
  // a refactor might pull the "modelType" and "modelID" out of cluster.
  
  Bulb(int clusterPosition, String modelType, int modelID, Vec3D modelCenter, LXTransform transform, float x, float y, float z, float rx, float ry, float rz) {
      
    // LXBulb has global X,Y,Z
    super(modelCenter.x + x, modelCenter.y + y,modelCenter.z + z);
        
    // This is an LX transform. I guess it gets applied to the LX point?
    transform.push();
    transform.translate(this.x, this.y, this.z);
    transform.rotateX(rx);
    transform.rotateY(ry);
    transform.rotateZ(rz);
    
    this.clusterPosition = clusterPosition;
    
    this.modelType = modelType;
    this.modelCenter = modelCenter;
    this.modelID = modelID;
    
    // System.out.format(" creating bulb(%f,%f,%f)\n",x,y,z);
    //System.out.format(" creating bulb(%f,%f,%f): index %d clusterpos %d\n",x,y,z,index,this.clusterPosition);

    this.rx = rx;
    this.ry = ry;
    this.rz = rz;
    this.tx = x;
    this.ty = y;
    this.tz = z;

    this.r = (float)Point2D.distance(x, y, 0, 0);
    this.theta = 180 + 180/Utils.PI*Utils.atan2(z, x);
    this.cylinderPoint = new Vec2D(this.theta, this.ty);
    
    transform.pop();
  }

  void resetTransform() {
    transformedTheta = theta;
    transformedY = y;
  }

  void didTransform() {
    transformedCylinderPoint = new Vec2D(transformedTheta, transformedY);
  }
}

abstract class Layer extends LXLayer {

  protected final Model model;

  Layer(LX lx) {
    super(lx);
    model = (Model)lx.model;
  }
}

abstract class ModelTransform extends Effect {
  ModelTransform(LX lx) {
    super(lx);

    model.addModelTransform(this);
  }

  public void run(double deltaMs) {}

  abstract void transform(Model model);
}

class ModelTransformTask implements LXLoopTask {

  protected final Model model;

  ModelTransformTask(Model model) {
    this.model = model;
  }

  public void loop(double deltaMs) {
    model.runTransforms();
  }
}
