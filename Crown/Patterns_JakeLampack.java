import heronarts.lx.LX;
import heronarts.lx.modulator.SawLFO;

class AcidTrip extends TSPattern {
  
  final SawLFO trails = new SawLFO(360, 0, 7000);
  
  AcidTrip(LX lx) {
    super(lx);

    addModulator(trails).start();
  }
    
  public void run(double deltaMs) {
    if (getChannel().getFader().getNormalized() == 0) return;
   
    for (Bulb bulb : model.bulbs) {
      colors[bulb.index] = lx.hsb(
        Utils.abs(model.cy - bulb.transformedY) + Utils.abs(model.cy - bulb.transformedTheta) + trails.getValuef() % 360,
        100,
        100
      );
    }
  }
}
