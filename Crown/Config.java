final class Config {

  static final boolean autoplayBMSet = true;

  // One or the other not both for now at least... maybe it can handle it :-/ ? 
  static final boolean enableAPC40 = true;
  static final boolean enableLaunchpad = false;

  static final boolean enableOutputBigtree = true;

  // When running visually in processing, if you have enough horsepower,
  // turn this on and you will see the cool cutouts.
  static final boolean enableSTLDisplay = false;

  // when it comes up default, should it output packets?
  static final boolean enableLiveOutput = false;

  static final String CLUSTER_CONFIG_FILE = "data/crown_clusters.json";
  
  static final String FENCE_STL_FILE = "data/fence.stl";

  static final String DEFAULT_PLAYLIST = "data/Burning Man Playlist.json";

}
