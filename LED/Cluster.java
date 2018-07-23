import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// We have a 1-1 mapping of Cluster to Model in this case.
// Therefore, simplify to having a cluster interface
public interface Cluster {
    
    public String getIpAddress();
    
    public List<Bulb> getBulbs();
    
}