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

import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class RingularityPattern extends TSPattern {

	private class Ring {

		public SinLFO ringMin;
		public SinLFO ringMax;
		public float speed;

		public Ring() {
			speed = Utils.random(2000, 8000);
			ringMin = new SinLFO(-50, 450, speed);
			ringMax = new SinLFO(Utils.random(-25, 25), 450, speed);
			addModulator(ringMin).start();
			addModulator(ringMax).start();

		}
	}

	private class FenceRing {
		public SawLFO ringMin;
		public float ringMax;
		public float speed;

		public FenceRing() {
			speed = Utils.random(2000, 8000);
			ringMin = new SawLFO(0, 360, speed);
			ringMax = ringMin.getValuef() + Utils.random(10, 30);
			addModulator(ringMin).start();

		}
	}

	final int TOWER_BULBS = 36;
	final int TOWER_CHANNELS = 12;

	final int FENCE_BULBS = 60;
	final int FENCE_CHANNELS = 12;
	
	private Ring ring1;
	private List<Ring> tower0Rings = new ArrayList<Ring>();
	private List<Ring> tower1Rings = new ArrayList<Ring>();
	private List<Ring> tower2Rings = new ArrayList<Ring>();
	private List<Ring> tower3Rings = new ArrayList<Ring>();
	private List<List> allTowers = new ArrayList<>();
	private int tower0size;
	private int tower1size;
	private int tower2size;
	private int tower3size;
	private List<FenceRing> fenceRings = new ArrayList<FenceRing>();
	private int fenceSize; 


	Random random;
	int randomNum;
	
	//final SinLFO satSin = new SinLFO(50, 100, 967);
	final SinLFO hueSin = new SinLFO(180, 255, 2000);
	
	RingularityPattern(LX lx) {
		super(lx);
		random = new Random();
		randomNum = random.nextInt((3 - 1) + 1) + 1;
		addModulator(hueSin).start();
		
		tower0size = random.nextInt((3 - 1) + 1) + 1;
		tower1size = random.nextInt((3 - 1) + 1) + 1;
		tower2size = random.nextInt((3 - 1) + 1) + 1;
		tower3size = random.nextInt((3 - 1) + 1) + 1;
		fenceSize = random.nextInt((3 - 1) + 1) + 1;

		for(int i = 0; i < tower0size; i++) {
			tower0Rings.add(new Ring());
		}
		for(int i = 0; i < tower1size; i++) {
			tower1Rings.add(new Ring());
		}
		for(int i = 0; i < tower2size; i++) {
			tower2Rings.add(new Ring());
		}
		for(int i = 0; i < tower3size; i++) {
			tower3Rings.add(new Ring());
		}
		for(int i = 0; i < fenceSize; i++) {
			fenceRings.add(new FenceRing());
		}
	}
	
	
	public void run(double deltaMs) {
		
		if (getChannel().getFader().getNormalized() == 0) return;
		
		for (Tower tower : model.towers) {
			for (Bulb bulb : tower.bulbs)

				if (tower.index == 0) {
					for(int i = 0; i < tower0Rings.size(); i ++)
					{
						setColor(bulb.index, lx.hsb((int)hueSin.getValuef(), 100, ((bulb.y > tower0Rings.get(i).ringMin.getValuef()) && (bulb.y < tower0Rings.get(i).ringMax.getValuef())) ? 100 : 0));
						if ((bulb.y > tower0Rings.get(i).ringMin.getValuef()) && (bulb.y < tower0Rings.get(i).ringMax.getValuef())) break;
					}
				}
				else if (tower.index == 1) {
					for(int i = 0; i < tower1Rings.size(); i ++)
					{
						setColor(bulb.index, lx.hsb((int)hueSin.getValuef(), 100, ((bulb.y > tower1Rings.get(i).ringMin.getValuef()) && (bulb.y < tower1Rings.get(i).ringMax.getValuef())) ? 100 : 0));
						if ((bulb.y > tower1Rings.get(i).ringMin.getValuef()) && (bulb.y < tower1Rings.get(i).ringMax.getValuef())) break;
					}
				} 
				else if (tower.index == 2) {
					for(int i = 0; i < tower2Rings.size(); i ++)
					{
						setColor(bulb.index, lx.hsb((int)hueSin.getValuef(), 100, ((bulb.y > tower2Rings.get(i).ringMin.getValuef()) && (bulb.y < tower2Rings.get(i).ringMax.getValuef())) ? 100 : 0));
						if ((bulb.y > tower2Rings.get(i).ringMin.getValuef()) && (bulb.y < tower2Rings.get(i).ringMax.getValuef())) break;
					}
				}
				else if (tower.index == 3) {
					for(int i = 0; i < tower3Rings.size(); i ++)
					{
						setColor(bulb.index, lx.hsb((int)hueSin.getValuef(), 100, ((bulb.y > tower3Rings.get(i).ringMin.getValuef()) && (bulb.y < tower3Rings.get(i).ringMax.getValuef())) ? 100 : 0));
						if ((bulb.y > tower3Rings.get(i).ringMin.getValuef()) && (bulb.y < tower3Rings.get(i).ringMax.getValuef())) break;
					}
				}
			}
			for (Bulb bulb : model.fence.bulbs) {
				for(int i = 0; i < fenceRings.size(); i++)
				{
					setColor(bulb.index, lx.hsb((int)hueSin.getValuef(), 100, ((bulb.theta > (fenceRings.get(i).ringMin.getValuef() % 360)) && (bulb.theta < ((fenceRings.get(i).ringMin.getValuef() + fenceRings.get(i).ringMax) % 360)) ? 100 : 0)));
						if ((bulb.theta > (fenceRings.get(i).ringMin.getValuef()) % 360) && (bulb.theta < ((fenceRings.get(i).ringMin.getValuef() + fenceRings.get(i).ringMax) % 360))) break;
				}
			}
		}
	}
