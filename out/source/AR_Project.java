import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import KinectProjectorToolkit.*; 
import javax.swing.JFrame; 
import org.openkinect.freenect.*; 
import org.openkinect.freenect2.*; 
import org.openkinect.processing.*; 
import org.processing.wiki.triangulate.*; 
import processing.sound.*; 
import gab.opencv.*; 
import controlP5.*; 
import Jama.*; 
import java.awt.Rectangle; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class AR_Project extends PApplet {
















Kinect2 kinect;

FFT fft;
SoundFile musicFile;
int bands = 64;
float[] spectrum = new float[bands];
float[] spectrumb = new float[bands];
boolean skipBands = false;
int bandsToSkip = 0;

PImage ir, invIr,threshIr,flipIr,rgbImg;
float threshVal;
OpenCV opencv;
// Mat testMat;
ArrayList<Contour> contours;
ArrayList<Contour> polygons;
ArrayList<PVector> contPoints;

//Triangllll
ArrayList triangles = new ArrayList();

Rectangle conRect;
KinectProjectorToolkit kpt;
PVector[] depthMap;
final int DEPTH_WIDTH = 512;
final int DEPTH_HEIGHT = 424;
PVector testPoint, testPointP;
public void setup() {
  kpt = new KinectProjectorToolkit(this, 512, 424);
  kpt.loadCalibration("calibration.txt");
  depthMap = new PVector[512*424];
  kinect = new Kinect2(this);
  kinect.initDepth();
  kinect.initVideo();
  kinect.initRegistered();
  kinect.initIR();
  kinect.initDevice();
  contours = new ArrayList<Contour>();
  contPoints = new ArrayList<PVector>();
  kpt.setDepthMapRealWorld(depthMap);
  depthMap = new PVector[kinect.depthWidth*kinect.depthHeight];
  opencv = new OpenCV(this,kinect.depthWidth,kinect.depthHeight); //512 424
  ir = kinect.getIrImage();
  testPoint = new PVector();
  testPointP = new PVector();

  fft = new FFT(this,bands);
  //"KeepItRollin.mp3"
  //"DECOY.wav"
  musicFile = new SoundFile(this,"DECOY.wav"
  );
  musicFile.play();
  fft.input(musicFile);

}
public void settings() {

  size(1920, 1080, P2D);
  fullScreen(2);
}

public void draw() {
  background(0);
  depthMap = depthMapRealWorld();  
  ir = kinect.getIrImage(); //Infrared Image
  // rgbImg = kinect.getVideoImage();
 
  flipIr = ir.copy(); //Make a copy of IR image
  mirrorImage(flipIr); 
  // image(flipIr,1920-512,0); //Display Mirrored IR Image
  
  invIr = ir.copy(); //Make another copy of IR image
  // invIr.filter(INVERT); //Invert the IR image

  contours = getContoursFromIR(invIr);

  contPoints = getPointsFromContours(contours);
  addStartPoints(contPoints);
  stroke(0,255,0);
  // fill(0,255,0);
  noFill();

  if(contPoints.size() > 4) {
    triangles = Triangulate.triangulate(contPoints);
    fft.analyze(spectrum);
    for (int i = 0; i < bands; i++) {
      spectrumb[i] = (spectrum[i] + spectrum[bands-1 -i])/2;
      // spectrumb[i] = spectrum[0];
    }
    int bandFactor = bands / triangles.size();
    if ( bands % triangles.size() != 0) {
      skipBands = true;
      bandsToSkip = bands % triangles.size();
    }
    else {
      skipBands = false;
    }
    // print(bands % triangles.size() + "\n");
    // int rectSpace = 1920 / triangles.size(); //for doing those sound bars...
    for (int i = 0; i < triangles.size(); i++) {
      Triangle t = (Triangle)triangles.get(i);
      //Get sum of freqs over intervals
      float sumFill = 0;
      
      for (int b = i * bandFactor; b < (i+1) * bandFactor; b++) {
        if(skipBands && i == triangles.size() - 1) {
          sumFill = sumFill + spectrumb[b + bandsToSkip];
        }
        else {
          sumFill = sumFill + spectrumb[b];
        }
        
      }
      // push();
      // noStroke();
      // fill(0,50,100);
      // float rectY = map(sumFill,0,1,1080,0);
      // rect(i*rectSpace,rectY,rectSpace,1080-rectY);
      // pop();
      push();
      stroke(0,255,0);
      // float fillness = spectrum[i*bandFactor] * 255;
      float fillness = sumFill * 255;
      fill(fillness);
      triangle(t.p1.x, t.p1.y, t.p2.x, t.p2.y, t.p3.x, t.p3.y);
      pop();
      //Centroid
      // float xav = (t.p1.x + t.p2.x + t.p3.x) / 3;
      // float yav = (t.p1.y + t.p2.y + t.p3.y) / 3;
      // text(i,xav,yav);
      // vertex(t.p1.x, t.p1.y);
      // vertex(t.p2.x, t.p2.y);
      // vertex(t.p3.x, t.p3.y);
    }
  }

  
}




// all functions below used to generate depthMapRealWorld point cloud
public PVector[] depthMapRealWorld() {
  int[] depth = kinect.getRawDepth();
  int skip = 1;
  for (int y = 0; y < kinect.depthHeight; y+=skip) {
    for (int x = 0; x < kinect.depthWidth; x+=skip) {
        int offset = x + y * kinect.depthWidth;
        //calculate the x, y, z camera position based on the depth information
        PVector point = depthToPointCloudPos(x, y, depth[offset]);
        depthMap[kinect.depthWidth * y + x] = point;
      }
    }
    return depthMap;
}

//calculte the xyz camera position based on the depth data
public PVector depthToPointCloudPos(int x, int y, float depthValue) {
  PVector point = new PVector();
  point.z = (depthValue);// / (1.0f); // Convert from mm to meters
  point.x = ((x - CameraParams.cx) * point.z / CameraParams.fx);
  point.y = ((y - CameraParams.cy) * point.z / CameraParams.fy);
  return point;
}

//camera information based on the Kinect v2 hardware
static class CameraParams {
  static float cx = 254.878f;
  static float cy = 205.395f;
  static float fx = 365.456f;
  static float fy = 365.456f;
  static float k1 = 0.0905474f;
  static float k2 = -0.26819f;
  static float k3 = 0.0950862f;
  static float p1 = 0.0f;
  static float p2 = 0.0f;
}

public void mirrorImage(PImage image) {
  image.loadPixels();
  for (int h = 0; h < image.height; h++) {
      for (int r = 0; r < image.width / 2; r++) {
        int offset = h * image.width + r;
        int temp = image.pixels[offset];
        image.pixels[offset] = image.pixels[h*image.width + (image.width - r - 1)];
        image.pixels[h*image.width + (image.width - r - 1)] = temp;
      }
    }
    image.updatePixels();
}

public ArrayList<Contour> getContoursFromIR(PImage IRImage) {
  IRImage.filter(INVERT);
  float threshVal = 0.825f;
  IRImage.filter(THRESHOLD,threshVal);
  IRImage.loadPixels();
  
  cleanBoundsOfImage(IRImage,152,392,130,258); //shit confused me with mirroring
  
  // image(IRImage,513,0);
  stroke(0,255,0);
  noFill();
  // rect(1920-512+120,130,360-120,258-130); //drawing rectangle over ir image of processed area
  

  IRImage.updatePixels();
  opencv.loadImage(IRImage);
  ArrayList<Contour> allContours = opencv.findContours();
  if (allContours.size() > 20) {
    allContours.clear();
  }
  else {
    for (int i = allContours.size() - 1; i >= 0; i--) {
      Contour contour = allContours.get(i);
      //filter shapes that are too big or small
      if (contour.area() < 5 || contour.area() > 22) {
        allContours.remove(i);
      }
    }
  }
  return allContours;
}

public void cleanBoundsOfImage(PImage image, int xLo, int xHi, int yLo, int yHi) {
  for (int h = 0; h < image.height; h++) {
    for (int r = 0; r < image.width; r++) {
      int offset = h * image.width + r;
      // The thresholds below were rougly approximated
      // This could be done better...
      if ((h > yLo && h < yHi) && (r > xLo && r < xHi)) {
        image.pixels[offset] = image.pixels[offset];
      }
      else {
        image.pixels[offset] = 0;
      }
    }
  }
}

public void mousePressed(){
  print("x:" + mouseX);
  print("y:" + mouseY);
}
// threshVal = map(mouseY,1080,0,0,1); //This was for identifyinga threshold to use

public ArrayList<PVector> getPointsFromContours(ArrayList<Contour> contours) {
  float contArea = 0;
  ArrayList<PVector> contourPoints = new ArrayList<PVector>();
  if (contours.size()>0) {
    for (Contour contour:contours) {
      contArea = contArea + contour.area();
      stroke(0,255,0);
      // contour.draw();
      conRect = contour.getBoundingBox();
      testPoint = new PVector(conRect.x + conRect.width/2,conRect.y + conRect.height/2);
      int idx = kinect.depthWidth * (int) testPoint.y + (int) testPoint.x;
      testPointP = kpt.convertKinectToProjector(depthMap[idx]);
      contourPoints.add(testPointP);
      // fill(0,255,0);
      // ellipse(round(testPointP.x), round(testPointP.y), 20, 20);  
      // text("Area:" + contour.area(),testPointP.x + 20,testPointP.y);
    }
    // textSize(40);
    // text("CONTOURS:" + contours.size(),1500,600);
    // text("AVG AREA:" + contArea / contours.size(),1500,700);
  }
  return contourPoints;
}

public void addStartPoints(ArrayList<PVector> contPoints) {
  testPoint = new PVector(0,0);
  contPoints.add(testPoint);
  testPoint = new PVector(0,1080);
  contPoints.add(testPoint);
  testPoint = new PVector(1920,0); 
  contPoints.add(testPoint);
  testPoint = new PVector(1920,1080);
  contPoints.add(testPoint);

  testPoint = new PVector(0,1080/2);
  contPoints.add(testPoint);
  testPoint = new PVector(1920/2,0);
  contPoints.add(testPoint);
  testPoint = new PVector(1920,1080/2);
  contPoints.add(testPoint);
  testPoint = new PVector(1920/2,1080);
  contPoints.add(testPoint);

}

public int getNumFreqBands(int numTriangles) {
  int freqs = 2;
  while(numTriangles > freqs) {
    freqs = freqs * 2;
  }
  return freqs;
}


//Drawing lines between all points
// if (contPoints.size() > 4) {
  //   for (int i = 0;  i < contPoints.size(); i++) {
  //     PVector pointA = contPoints.get(i);
  //     for (int j = 0;  j < contPoints.size(); j++) {
  //       if (i != j) {
  //         PVector pointB = contPoints.get(j);
  //         // float dist = pointA.dist(pointB);
  //         line(pointA.x,pointA.y,pointB.x,pointB.y);
  //       }
  //     }
  //   }
  // }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "AR_Project" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
