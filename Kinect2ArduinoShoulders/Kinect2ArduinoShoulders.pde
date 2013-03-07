import SimpleOpenNI.*;
SimpleOpenNI  kinect;
import processing.serial.*;
Serial port;

void setup() { 
  size(640, 480);

  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  kinect.setMirror(true);
  
  println(Serial.list());
String portName = Serial.list()[0];
port = new Serial(this, portName, 9600);

  
}

void draw() {
  kinect.update();
  PImage depth = kinect.depthImage();
  image(depth, 0, 0);

  IntVector userList = new IntVector();
  kinect.getUsers(userList);

  if (userList.size() > 0) {
    int userId = userList.get(0);

    if ( kinect.isTrackingSkeleton(userId)) {
      // get the positions of the three joints of our arm
     
     /////////////////hands
     
      PVector rightHand = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_RIGHT_HAND,
                                      rightHand);
                                      
        PVector leftHand = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_LEFT_HAND,
                                      leftHand);                                

    
    
    ///////////////////elbows
      PVector rightElbow = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_RIGHT_ELBOW,
                                      rightElbow);
                                      
                                      
                                      
      PVector leftElbow = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_LEFT_ELBOW,
                                      leftElbow);




////////////////////////shoulders
      PVector rightShoulder = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_RIGHT_SHOULDER,
                                      rightShoulder);
                                      
      PVector leftShoulder = new PVector();
       kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_LEFT_SHOULDER,
                                      leftShoulder);
      
                                      
                                      

      // we need right hip to orient the shoulder angle
      PVector rightHip = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_RIGHT_HIP,
                                      rightHip);

      // reduce our joint vectors to two dimensions
      PVector rightHand2D = new PVector(rightHand.x, rightHand.y); 
      PVector rightElbow2D = new PVector(rightElbow.x, rightElbow.y);
      PVector rightShoulder2D = new PVector(rightShoulder.x,
                                            rightShoulder.y);
      PVector rightHip2D = new PVector(rightHip.x, rightHip.y);

      // calculate the axes against which we want to measure our angles
      PVector torsoOrientation =
        PVector.sub(rightShoulder2D, rightHip2D); 
      PVector upperArmOrientation =
        PVector.sub(rightElbow2D, rightShoulder2D);

  /////////////////////////////////////////////////////////LEFT ORIENTATION
       PVector leftHip = new PVector();
      kinect.getJointPositionSkeleton(userId,
                                      SimpleOpenNI.SKEL_LEFT_HIP,
                                      leftHip);

      // reduce our joint vectors to two dimensions
      PVector leftHand2D = new PVector(leftHand.x, leftHand.y); 
      PVector leftElbow2D = new PVector(leftElbow.x, leftElbow.y);
      PVector leftShoulder2D = new PVector(leftShoulder.x,
                                            leftShoulder.y);
      PVector leftHip2D = new PVector(leftHip.x, leftHip.y);

    
    
      // calculate the axes against which we want to measure our angles
      PVector torsoOrientationLeft =
        PVector.sub(leftShoulder2D, leftHip2D); 
      PVector upperArmOrientationLeft =
        PVector.sub(leftElbow2D, leftShoulder2D);
  
  
  
      // calculate the angles between our joints
      float shoulderAngleRight = angleOf(rightElbow2D,      
                                    rightShoulder2D,
                                    torsoOrientation);
                                    
       float shoulderAngleLeft = angleOf(leftElbow2D,      
                                    leftShoulder2D,
                                    torsoOrientationLeft);   
         println(shoulderAngleLeft);
         
     
     
      float elbowAngle    = angleOf(rightHand2D,
                                    rightElbow2D,
                                    upperArmOrientation);

    
      // show the angles on the screen for debugging
      fill(255,0,0);
      scale(3);
      text("right shoulder: " + int(shoulderAngleRight) + "\n" +
           " left shoulder: " + int(shoulderAngleLeft), 20, 20);
           
           byte out[] = new byte[2];
out[0] = byte(shoulderAngleRight);
out[1] = byte(shoulderAngleLeft);
port.write(out);
           
    }
  }
}

float angleOf(PVector one, PVector two, PVector axis) {
  PVector limb = PVector.sub(two, one);
  return degrees(PVector.angleBetween(limb, axis));
}

// user-tracking callbacks!
void onNewUser(int userId) {
  println("start pose detection");
  kinect.startPoseDetection("Psi", userId);
}

void onEndCalibration(int userId, boolean successful) {
  if (successful) {
    println("  User calibrated !!!");
    kinect.startTrackingSkeleton(userId);
  }
  else {
    println("  Failed to calibrate user !!!");
    kinect.startPoseDetection("Psi", userId);
  }
}

void onStartPose(String pose, int userId) {
  println("Started pose for user");
  kinect.stopPoseDetection(userId);
  kinect.requestCalibrationSkeleton(userId, true);
}
