import java.util.List;

import heronarts.lx.LX;
import heronarts.lx.color.LXColor;
import heronarts.lx.modulator.SinLFO;
import heronarts.lx.parameter.BooleanParameter;
import heronarts.lx.parameter.DiscreteParameter;

class MappingTool extends Effect {

  final List<ClusterConfig> clusterConfig;

  final SinLFO strobe = new SinLFO(20, 100, 1000);
  
  final DiscreteParameter clusterIndex;
  final BooleanParameter showBlanks = new BooleanParameter("BLANKS", false);

  MappingTool(LX lx, List<ClusterConfig> clusterConfig) {
    super(lx);
    this.clusterConfig = clusterConfig;
    clusterIndex = new DiscreteParameter("CLUSTER", clusterConfig.size());
    addModulator(strobe).start();
    addLayer(new MappingLayer());
  }
  
  ClusterConfig getConfig() {
    return clusterConfig.get(clusterIndex.getValuei());
  }
  
  Cluster getCluster() {
    return model.clustersByIp.get(getConfig().ipAddress);
  }

  public void run(double deltaMs) {
  }
  
  class MappingLayer extends Layer {
    
    MappingLayer() {
      super(MappingTool.this.lx);
    }
    
    public void run(double deltaMs) {
      if (isEnabled()) {
        List<Bulb> bulbs = getCluster().getBulbs();
        for (Bulb bulb : bulbs) {
          blendColor(bulb.index, lx.hsb(0, 0, strobe.getValuef()), LXColor.Blend.ADD);
        }
      }
    }
  }
}
