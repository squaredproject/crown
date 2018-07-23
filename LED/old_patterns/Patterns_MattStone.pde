import heronarts.lx.LX;

import codeanticode.syphon.*;

PGraphics buffer;
PImage imgbuffer;
SyphonClient client;

class SyphonPattern extends TSPattern {

  int x, y, z, buffWidth, buffHeight = 0;
  float xscale, yscale = 0f;
  int[] xpoints, ypoints;

  final DiscreteParameter getWidth = new DiscreteParameter("GW", 1, 20);
  final DiscreteParameter mode = new DiscreteParameter("MODE", 1, 5);

  SyphonPattern(LX lx, PApplet applet) {
    super(lx);
    addParameter(getWidth);
    addParameter(mode);
    client = new SyphonClient(applet, "Modul8", "Main View");
    xpoints = new int[model.bulbs.size()];
    ypoints = new int[model.bulbs.size()];
  }

  void generateMap(int buffWidth, int buffHeight) {
    this.xscale = buffWidth / model.xRange;
    this.yscale = buffHeight / model.yRange;
    int bulbIdx = 0;    
    for (Bulb bulb : model.bulbs) {
      xpoints[bulbIdx] = int((bulb.cx - model.xMin) * this.xscale);
      ypoints[bulbIdx] = buffHeight - int((bulb.cy - model.yMin) * this.yscale);    
      bulbIdx++;
    }
  }

  private int mode1(Bulb bulb, int bulbIdx) {    
    return weighted_get(imgbuffer, int(this.buffWidth * (bulb.transformedTheta / 360.0)), this.buffHeight - int(this.buffHeight * (bulb.transformedY/model.yMax)), getWidth.getValuei());
  }
  
  private int mode2(Bulb bulb, int bulbIdx) {
    boolean reverse = false;
    if (bulb.transformedTheta > (360.0 / 2))
      reverse = true;
    if (reverse) {
      return weighted_get(imgbuffer, int(this.buffWidth * ((((360.0 - bulb.transformedTheta) * 2)) / 360.0)), this.buffHeight - int(this.buffHeight * (bulb.transformedY/model.yMax)), getWidth.getValuei());      
    }
    return weighted_get(imgbuffer, int(this.buffWidth * ((bulb.transformedTheta * 2.0) / 360.0)), this.buffHeight - int(this.buffHeight * (bulb.transformedY/model.yMax)), getWidth.getValuei());
  }
  
  private int mode3(Bulb bulb, int bulbIdx) {
    return weighted_get(imgbuffer, xpoints[bulbIdx], ypoints[bulbIdx], getWidth.getValuei());
  }
        
  public void run(double deltaMs) {
    if (client.available()) {

      buffer = client.getGraphics(buffer);
      imgbuffer = buffer.get();
      this.buffWidth = buffer.width;
      this.buffHeight = buffer.height;
      if (this.xscale == 0) {
        generateMap(buffer.width, buffer.height);
      }
      int bulbIdx = 0;
      int c = 0;
      for (Bulb bulb : model.bulbs) {
        switch (mode.getValuei()) {
          case 1: c = mode1(bulb, bulbIdx);
                  break;
          case 2: c = mode2(bulb, bulbIdx);
                  break;    
          case 3: c = mode3(bulb, bulbIdx);
                  break;      
        }
        
        setColor(bulb.index, c);
        bulbIdx++;
      }
    }
  }
  
  private boolean restoreThreaded = false;
  private int syphonCount = 0;
  
  public void onActive() {
    if (syphonCount == 0) {
      if (restoreThreaded = lx.engine.isThreaded()) {
        println("Turning off threading for Syphon");
        lx.engine.setThreaded(false);
      }
    }
    ++syphonCount; 
  }
  
  public void onInactive() {
    --syphonCount;
    if ((syphonCount == 0) && restoreThreaded) {
      println("Restoring threading from Syphon");
      lx.engine.setThreaded(true);
    }
  }
}


int weighted_get(PImage imgbuffer, int xpos, int ypos, int radius) {
  int h, s, b;
  int xoffset, yoffset;
  int pixels_counted;

  int thispixel;


  h = s = b = pixels_counted = 0;

  for (xoffset=-radius; xoffset<radius; xoffset++) {
    for (yoffset=-radius; yoffset<radius; yoffset++) {

      pixels_counted ++;
      thispixel = imgbuffer.get(xpos + xoffset, ypos + yoffset);

      h += hue(thispixel);
      s += saturation(thispixel);
      b += brightness(thispixel);
    }
  }
  return color(h/pixels_counted, s/pixels_counted, b/pixels_counted);
}
