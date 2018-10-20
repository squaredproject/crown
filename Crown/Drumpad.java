
interface Drumpad {
  public void padTriggered(int row, int col, float velocity);
  public void padReleased(int row, int col);
}

class TSDrumpad implements Drumpad {
  
  Triggerable[][] triggerables = null;
  
  public void padTriggered(int row, int col, float velocity) {

    if (triggerables == null) {
      System.out.println("Drumpad Triggered without Triggerables, check Config.java");
      return;
    }
    //System.out.printf("Drumpad Triggered with row %d col %d\n",row,col);

    if ((row < triggerables.length) && (col < triggerables[row].length)) {
      //System.out.printf("Drumpad calling Triggerable with row %d col %d\n",row,col);
      triggerables[row][col].onTriggered(velocity);
    }
    else {
      System.out.printf(" Drumpad: no effect listed for row %d col %d\n",row,col);
    }
    
  }
  
  public void padReleased(int row, int col) {

    //System.out.printf("Drumpad Triggered Released with row %d col %d\n",row,col);

    if (triggerables != null && row < triggerables.length && col < triggerables[row].length) {
      triggerables[row][col].onRelease();
    }
  }
}
