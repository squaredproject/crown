import heronarts.lx.LX;
import heronarts.lx.LXUtils;
import heronarts.lx.color.LXColor;
import heronarts.lx.transform.LXProjection;
import heronarts.lx.transform.LXVector;
import heronarts.lx.modulator.DampedParameter;
import heronarts.lx.modulator.SawLFO;
import heronarts.lx.modulator.SinLFO;
import heronarts.lx.parameter.BasicParameter;
import heronarts.lx.parameter.DiscreteParameter;
import heronarts.lx.parameter.LXParameter;


/*
* this pattern depends on ClusterY thus must be retooled.
** todo.
*/

/*

class DoubleHelix extends TSPattern {
  
  final SinLFO rate = new SinLFO(400, 3000, 11000);
  final SawLFO theta = new SawLFO(0, 180, rate);
  final SinLFO coil = new SinLFO(0.2, 2, 13000);
  
  DoubleHelix(LX lx) {
    super(lx);
    addModulator(rate).start();
    addModulator(theta).start();
    addModulator(coil).start();
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    for (Bulb bulb : model.bulbs) {
      float coilf = coil.getValuef() * (bulb.cy - model.cy);
      colors[bulb.index] = lx.hsb(
        lx.getBaseHuef() + .4f*Utils.abs(bulb.transformedY - model.cy) +.2f* Utils.abs(bulb.transformedTheta - 180),
        100,
        Utils.max(0, 100 - 2*LXUtils.wrapdistf(bulb.transformedTheta, theta.getValuef() + coilf, 180))
      );
    }
  }
}
*/

class ColoredLeaves extends TSPattern {
  
  private SawLFO[] movement;
  private SinLFO[] bright;
  
  ColoredLeaves(LX lx) {
    super(lx);
    movement = new SawLFO[3];
    for (int i = 0; i < movement.length; ++i) {
      movement[i] = new SawLFO(0, 360, 60000 / (1 + i));
      addModulator(movement[i]).start();
    }
    bright = new SinLFO[5];
    for (int i = 0; i < bright.length; ++i) {
      bright[i] = new SinLFO(100, 0, 60000 / (1 + i));
      addModulator(bright[i]).start();
    }
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    for (Bulb bulb : model.bulbs) {
      colors[bulb.index] = lx.hsb(
        (360 + movement[bulb.index  % movement.length].getValuef()) % 360,
        100,
        bright[bulb.index % bright.length].getValuef()
      );
    }
  }
}

class SeeSaw extends TSPattern {
  
  final LXProjection projection = new LXProjection(model);

  final SinLFO rate = new SinLFO(2000, 11000, 19000);
  final SinLFO rz = new SinLFO(-15, 15, rate);
  final SinLFO rx = new SinLFO(-70, 70, 11000);
  final SinLFO width = new SinLFO(1*Geometry.FEET, 8*Geometry.FEET, 13000);

  final BasicParameter bgLevel = new BasicParameter("BG", 25, 0, 50);
  
  SeeSaw(LX lx) {
    super(lx);
    addModulator(rate).start();
    addModulator(rx).start();
    addModulator(rz).start();
    addModulator(width).start();
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    projection
      .reset()
      .center()
      .rotate(rx.getValuef() * Utils.PI / 180, 1, 0, 0)
      .rotate(rz.getValuef() * Utils.PI / 180, 0, 0, 1);
    for (LXVector v : projection) {
      colors[v.index] = lx.hsb(
        (lx.getBaseHuef() + Utils.min(120, Utils.abs(v.y))) % 360,
        100,
        Utils.max(bgLevel.getValuef(), 100 - (100/(1*Geometry.FEET))*Utils.max(0, Utils.abs(v.y) - 0.5f*width.getValuef()))
      );
    }
  }
}

class Twister extends TSPattern {

  final SinLFO spin = new SinLFO(0, 5*360, 16000);
  
  float coil(float basis) {
    return Utils.sin(basis*Utils.TWO_PI - Utils.PI);
  }
  
  Twister(LX lx) {
    super(lx);
    addModulator(spin).start();
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    float spinf = spin.getValuef();
    float coilf = 2*coil(spin.getBasisf());
    for (Bulb bulb : model.bulbs) {
      float wrapdist = LXUtils.wrapdistf(bulb.transformedTheta, spinf + (model.yMax - bulb.transformedY)*coilf, 360);
      float yn = (bulb.transformedY / model.yMax);
      float width = 10 + 30 * yn;
      float df = Utils.max(0, 100 - (100 / 45) * Utils.max(0, wrapdist-width));
      colors[bulb.index] = lx.hsb(
        (lx.getBaseHuef() + .2f*bulb.transformedY - 360 - wrapdist) % 360,
        Utils.max(0, 100 - 500*Utils.max(0, yn-.8f)),
        df
      );
    }
  }
}

/*
** this depends on the Cluster location which no longer exists
*/

/*
class SweepPattern extends TSPattern {
  
  final SinLFO speedMod = new SinLFO(3000, 9000, 5400);
  final SinLFO yPos = new SinLFO(model.yMin, model.yMax, speedMod);
  final SinLFO width = new SinLFO("WIDTH", 2*Geometry.FEET, 20*Geometry.FEET, 19000);
  
  final SawLFO offset = new SawLFO(0, Utils.TWO_PI, 9000);
  
  final BasicParameter amplitude = new BasicParameter("AMP", 10*Geometry.FEET, 0, 20*Geometry.FEET);
  final BasicParameter speed = new BasicParameter("SPEED", 1, 0, 3);
  final BasicParameter height = new BasicParameter("HEIGHT", 0, -300, 300);
  final SinLFO amp = new SinLFO(0, amplitude, 5000);
  
  SweepPattern(LX lx) {
    super(lx);
    addModulator(speedMod).start();
    addModulator(yPos).start();
    addModulator(width).start();
    addParameter(amplitude);
    addParameter(speed);
    addParameter(height);
    addModulator(amp).start();
    addModulator(offset).start();
  }
  
  public void onParameterChanged(LXParameter parameter) {
    super.onParameterChanged(parameter);
    if (parameter == speed) {
      float speedVar = 1/speed.getValuef();
      speedMod.setRange(9000 * speedVar,5400 * speedVar);
    }
  }
  
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    for (Bulb bulb : model.bulbs) {
      float yp = yPos.getValuef() + amp.getValuef() * Utils.sin((bulb.cx - model.cx) * .01f + offset.getValuef());
      colors[bulb.index] = lx.hsb(
        (lx.getBaseHuef() + Utils.abs(bulb.x - model.cx) * .2f +  bulb.cz*.1f + bulb.cy*.1f) % 360,
        Utils.constrain(Utils.abs(bulb.transformedY - model.cy), 0, 100),
        Utils.max(0, 100 - (100/width.getValuef())*Utils.abs(bulb.cy - yp - height.getValuef()))
      );
    }
  }
}
*/

class DiffusionTestPattern extends TSPattern {
  
  final BasicParameter hue = new BasicParameter("HUE", 0, 360);
  final BasicParameter sat = new BasicParameter("SAT", 1);
  final BasicParameter brt = new BasicParameter("BRT", 1);
  final BasicParameter spread = new BasicParameter("SPREAD", 0, 360);
  
  DiffusionTestPattern(LX lx) {
    super(lx);
    addParameter(hue);
    addParameter(sat);
    addParameter(brt);
    addParameter(spread);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    setColors(LXColor.BLACK);
    for (int i = 0; i < 12; ++i) {
      colors[i] = lx.hsb(
        (hue.getValuef() + (i / 4) * spread.getValuef()) % 360,
        sat.getValuef() * 100,
        Utils.min(100, brt.getValuef() * (i+1) / 12.f * 200)
      );
    }
  }
}

/*
** todo: need a new test pattern that works
** with the objects I now have
*/


class T1Pattern extends TSPattern {

  // towers and fences have a different number of bulbs per channel
  final int TOWER_BULBS = 36;
  final int TOWER_CHANNELS = 12;
  
  final int FENCE_BULBS = 60;
  final int FENCE_CHANNELS = 12;
  
  final BasicParameter period = new BasicParameter("RATE", 200000, 25000, 2000000);
  final SawLFO towerBulbIndex = new SawLFO(0, TOWER_CHANNELS * TOWER_BULBS, period);
  final SawLFO fenceBulbIndex = new SawLFO(0, FENCE_CHANNELS * FENCE_BULBS, period);
  
  T1Pattern(LX lx) {
    super(lx);
    addModulator(towerBulbIndex).start();
    addModulator(fenceBulbIndex).start();
    addParameter(period);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;
    
//    System.out.printf("Making Test Pattern: fence BI %f tower BI %f \n",
//        fenceBulbIndex.getValuef(),towerBulbIndex.getValuef());

    for (Bulb bulb : model.bulbs) {
      if (bulb.modelType.equals("tower")) {
        setColor(bulb.index, lx.hsb(
            0,
            0,
            ( bulb.clusterPosition == (int) towerBulbIndex.getValuef() ) ? 100: 0
         ));
      }
      else if (bulb.modelType.equals("fence")) {
          setColor(bulb.index, lx.hsb(
            0,
            0,
            ( bulb.clusterPosition == (int) fenceBulbIndex.getValuef() ) ? 100: 0
          ));
      }
    }
  }
}


class T2Pattern extends TSPattern {

  // towers and fences have a different number of bulbs per channel
  final int TOWER_BULBS = 36;
  final int TOWER_CHANNELS = 12;
  
  final int FENCE_BULBS = 60;
  final int FENCE_CHANNELS = 12;
  
  // BasicParameter: start, lowlimit, highlimit
  final BasicParameter period = new BasicParameter("RATE", 10000, 1000, 30000);
  // period is time in millisecond it takes. Smaller is thus faster.
  final SawLFO fenceBulbIndex = new SawLFO(0, FENCE_CHANNELS, period);
  final SawLFO towerBulbIndex = new SawLFO(0, TOWER_CHANNELS, period);
  
  T2Pattern(LX lx) {
    super(lx);
    addModulator(fenceBulbIndex).start();
    addModulator(towerBulbIndex).start();
    addParameter(period);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;
    
    //System.out.printf("Making Test Pattern: towerBI %f fenceBI %f \n",towerBulbIndex.getValuef(), fenceBulbIndex.getValuef());

    for (Bulb bulb : model.bulbs) {
      if (bulb.modelType.equals("tower")) {
        setColor(bulb.index, lx.hsb(
            (lx.getBaseHuef() + (10*(bulb.clusterPosition % TOWER_BULBS))) % 360,
            100,
            ( bulb.clusterPosition / TOWER_BULBS == (int) towerBulbIndex.getValuef() ) ? 100: 0
         ));
      }
      else if (bulb.modelType.equals("fence")) {
          setColor(bulb.index, lx.hsb(
            (lx.getBaseHuef() + (5*(bulb.clusterPosition % FENCE_BULBS))) % 360,
            100,
            ( bulb.clusterPosition / FENCE_BULBS == (int) fenceBulbIndex.getValuef() ) ? 100: 0
          ));
      }
      else {
          System.out.println(" Test Pattern: found unknown model type "+bulb.modelType);
      }
    
    }
  }
}


class TestCluster extends TSPattern {
  final DiscreteParameter lightNo = new DiscreteParameter("LIGHT", 0, 19);
  final BasicParameter pos = new BasicParameter("POS", 0); 
  
  TestCluster(LX lx) {
    super(lx);
    addParameter(lightNo);
    addParameter(pos);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    // TODO: this has some pecular code using the number 17
    // and cluster range specifiers that don't seem to exist on the 
    // objects. We will likely have to write a TestPattern for
    // crown that makes sense, and doesn't hardcode the number 17.
    //
/*
    for (Cluster cluster : model.clusters) {
      for (Bulb bulb : cluster.bulbs) {
        if (lightNo.getValuei() >= 17) {
          float d = (lightNo.getValuei() == 17) ?
            ((bulb.transformedY - cluster.yMin) / cluster.yRange) :
            ((bulb.x - cluster.xMin) / cluster.xRange); 
          setColor(bulb, lx.hsb(
            lx.getBaseHuef(),
            100,
            Utils.max(0, 100 - 400*Utils.abs(d - pos.getValuef()))
          ));
        } else if ((bulb.clusterPosition == lightNo.getValuei()) ||
            (0 == lightNo.getValuei())) {
          setColor(bulb, lx.hsb(
            lx.getBaseHuef(),
            100,
            100
          )); 
        } else {
          setColor(bulb, 0);
        }
      }
    }
    */
  }
}

class ColorEffect extends Effect {
  
  final BasicParameter desaturation = new BasicParameter("WHT", 0);
  final BasicParameter hueShift = new BasicParameter("HUE", 0, 360);
  final BasicParameter sharp = new BasicParameter("SHRP", 0);
  final BasicParameter soft = new BasicParameter("SOFT", 0);
  final BasicParameter mono = new BasicParameter("MON", 0);
  final BasicParameter rainbow = new BasicParameter("ACID", 0);
  
  private final DampedParameter hueShiftd = new DampedParameter(hueShift, 180);
  private final DampedParameter rainbowd = new DampedParameter(rainbow, 1);
  
  private float[] hsb = new float[3];
  
  ColorEffect(LX lx) {
    super(lx);
    addParameter(desaturation);
    addParameter(hueShift);
    addParameter(sharp);
    addParameter(soft);
    addParameter(mono);
    addParameter(rainbow);
    
    addModulator(hueShiftd).start();
    addModulator(rainbowd).start();
  }
  
  protected void run(double deltaMs) {
    float desatf = desaturation.getValuef();
    float huef = hueShiftd.getValuef();
    float sharpf = sharp.getValuef();
    float softf = soft.getValuef();
    float monof = mono.getValuef();
    float rainbowf = rainbowd.getValuef();
    if (desatf > 0 || huef > 0 || sharpf > 0 || softf > 0 || monof > 0 || rainbowf > 0) {
      float pSharp = 1/(1-.99f*sharpf);
      for (int i = 0; i < colors.length; ++i) {
        float b = LXColor.b(colors[i]) / 100.f;
        float bOrig = b;
        if (sharpf > 0) {
          if (b < .5f) {
            b = Utils.pow(b, pSharp);
          } else {
            b = 1-Utils.pow(1-b, pSharp);
          }
        }
        if (softf > 0) {
          if (b > 0.5f) {
            b = Utils.lerp(b, 0.5f + 2 * (b-0.5f)*(b-0.5f), softf);
          } else {
            b = Utils.lerp(b, 0.5f * Utils.sqrt(2*b), softf);
          }
        }
        
        float h = LXColor.h(colors[i]);
        float bh = lx.getBaseHuef();
        if (rainbowf > 0) {
          h = bh + (h - bh) * (1+3*rainbowf);
          h = (h + 5*360) % 360;
        }
        if (Utils.abs(h - bh) > 180) {
          if (h > bh) {
            bh += 360;
          } else {
            h += 360;
          }
        }
        
        colors[i] = lx.hsb(
          (Utils.lerp(h, bh, monof) + huef) % 360,
          LXColor.s(colors[i]) * (1 - desatf),
          100*b
        );
      }
    }
  }
}

class ColorEffect2 extends ColorEffect {
  ColorEffect2(LX lx) {
    super(lx);
  }
}

/*
** BrianB wrote this very speicifc for Crown.
** Since crown has 4 towers, we should be able to rotate among the four.
*/

class RotateTower extends TSPattern {

  // towers and fences have a different number of bulbs per channel
  final int TOWERS = 4;

  
  final BasicParameter period = new BasicParameter("RATE", 2000, 10, 4000);
  final SawLFO towerIndex = new SawLFO(0, TOWERS, period);
  
  RotateTower(LX lx) {
    super(lx);
    addModulator(towerIndex).start();
    addParameter(period);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    //System.out.println(" tower index is: "+towerIndex.getValuef() );
    
//    System.out.printf("Making Test Pattern: fence BI %f tower BI %f \n",
//        fenceBulbIndex.getValuef(),towerBulbIndex.getValuef());

    for (Bulb bulb : model.bulbs) {
      if (bulb.modelType.equals("tower")) {
      	int b;
      	// selected tower?
      	if (bulb.modelID == (int)towerIndex.getValuef()) {
      		b = 100;
      	}
      	else {
      		b = 0;
      	}
        setColor(bulb.index, lx.hsb(
            0,
            0,
            b
         ));
      }

    }
  }
}

class ThrobTower extends TSPattern {

  // towers and fences have a different number of bulbs per channel
  final int TOWERS = 4;

  
  final BasicParameter period = new BasicParameter("RATE", 2000, 10, 4000);
  final BasicParameter hue = new BasicParameter("HUE",0,0,100);
  final BasicParameter sat = new BasicParameter("SAT",0,0,100);
  final SawLFO towerIndex = new SawLFO(0, TOWERS, period);
  
  ThrobTower(LX lx) {
    super(lx);
    addModulator(towerIndex).start();
    addParameter(period);
    addParameter(hue);
    addParameter(sat);
  }
  
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;

    //System.out.println(" tower index is: "+towerIndex.getValuef() );
    

   float b = towerIndex.getValuef();
   b = (b % 1);
   if ( b > 0.5f ) {
   	 b = 1.0f - b;
   }
   b = b * 100.0f;

    for (Bulb bulb : model.bulbs) {
      if (bulb.modelType.equals("tower")) {

        setColor(bulb.index, lx.hsb(
            hue.getValuef(),
            sat.getValuef(),
            (bulb.modelID == (int)towerIndex.getValuef()) ? b : 0 )
         );
      }

    }
  }
}

