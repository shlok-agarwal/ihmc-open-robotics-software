package atlas_msgs;

public interface WalkDemoGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/WalkDemoGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n# Goal\n# For Setting MultiStep Walking Commands\nHeader header\n\n# permissible values for behavior\nint32 STAND             =  0 # stand\nint32 USER              =  1 # disable AtlasSimInterface updates, rely on\n                             # /atlas/atlas_command or /atlas/joint_commands\nint32 FREEZE            =  2 # safety mode\nint32 STAND_PREP        =  3 # stand-prep (AtlasSimInterface documentation)\nint32 WALK              =  4 # multi-step walk\nint32 STEP              =  5 # single step walk\nint32 MANIPULATE        =  6 # stand and allows manipulation.\nint32 DEMO1             =  7 # walk in straight line\nint32 DEMO2             =  8 # figure 8\n\nint32 behavior                # can be one of\n                              # USER, FREEZE, STAND_PREP\n                              # WALK, STEP, STAND, MANIPULATE\n                              # DEMO1, DEMO2\n\n# multi_step walking trajectory parameters\natlas_msgs/AtlasBehaviorStepData[] steps\n\n# parameters for single_step behavior\natlas_msgs/AtlasBehaviorStepParams step_params\n\n# parameters for standing behavior\natlas_msgs/AtlasBehaviorStandParams stand_params\n\n# parameters for stand and manipulate\natlas_msgs/AtlasBehaviorManipulateParams manipulate_params\n\n# same k_effort as AtlasCommand\nuint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, \n                       # at run time, a double between 0 and 1 is obtained\n                       # by dividing by 255.0d.\n\n";
  static final int STAND = 0;
  static final int USER = 1;
  static final int FREEZE = 2;
  static final int STAND_PREP = 3;
  static final int WALK = 4;
  static final int STEP = 5;
  static final int MANIPULATE = 6;
  static final int DEMO1 = 7;
  static final int DEMO2 = 8;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getBehavior();
  void setBehavior(int value);
  java.util.List<atlas_msgs.AtlasBehaviorStepData> getSteps();
  void setSteps(java.util.List<atlas_msgs.AtlasBehaviorStepData> value);
  atlas_msgs.AtlasBehaviorStepParams getStepParams();
  void setStepParams(atlas_msgs.AtlasBehaviorStepParams value);
  atlas_msgs.AtlasBehaviorStandParams getStandParams();
  void setStandParams(atlas_msgs.AtlasBehaviorStandParams value);
  atlas_msgs.AtlasBehaviorManipulateParams getManipulateParams();
  void setManipulateParams(atlas_msgs.AtlasBehaviorManipulateParams value);
  org.jboss.netty.buffer.ChannelBuffer getKEffort();
  void setKEffort(org.jboss.netty.buffer.ChannelBuffer value);
}
