import java.util.List;

import heronarts.lx.output.DDPDatagram;

class Output {
  static DDPDatagram clusterDatagram(Cluster cluster) {
    List<Bulb> bulbs = cluster.getBulbs();
    int[] pointIndices = new int[bulbs.size()];
    int pi = 0;
    for (Bulb bulb : bulbs) {
        pointIndices[pi++] = bulb.index;
    }
    return new DDPDatagram(pointIndices);
  }
}
