package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.tools.thread.ThreadTools;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   //SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry();
   
   public RRTPlannerSolarPanelCleaning rrtPlanner;
   
   private double motionTime;
   
   // For debug
   public ArrayList<Pose> debugPose = new ArrayList<Pose>();
   
   public SolarPanelMotionPlanner(SolarPanel panel)
   {
      this.solarPanel = panel;
   }
   
   public enum CleaningMotion
   {
      ReadyPose,
      LinearCleaningMotion
   }
      
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return this.wholeBodyTrajectoryMessage;
   }      
   
   public double getMotionTime()
   {      
      return motionTime;
   }
      
   public boolean setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {      
      this.wholeBodyTrajectoryMessage.clear();
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.05, -Math.PI*0.2);
           
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         this.motionTime = 1.0;
         //motionFactory.setMessage(readyPose, Math.PI*0.0, 0.0, this.motionTime);
         //wholeBodyTrajectoryMessage = motionFactory.getWholeBodyTrajectoryMessage();
         
         debugPose.add(readyPose.getPose());
         
         break;
         
      case LinearCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
         
         SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.05, -Math.PI*0.3), 4.0);         
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 1.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.2), 4.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.2), 1.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 4.0);
                           
         this.motionTime = cleaningPath.getArrivalTime().get(cleaningPath.getArrivalTime().size()-1);
         PrintTools.info("motionTime :: "+this.motionTime);
         
         // *** RRT *** //         
         RRTNode1DTimeDomain.cleaningPath = cleaningPath;
         RRTNode1DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);
         
         
//         RRTNode3DTimeDomain node0 = new RRTNode3DTimeDomain();
//         node0.isValidNode();
//         
//
//         RRTNode3DTimeDomain node1 = new RRTNode3DTimeDomain(1.0, 0.8, 0/180*Math.PI, 0/180*Math.PI);
//         RRTNode3DTimeDomain node2 = new RRTNode3DTimeDomain(1.0, 0.9, 15/180*Math.PI, 0/180*Math.PI);
//         RRTNode3DTimeDomain node3 = new RRTNode3DTimeDomain(1.0, 0.7, -15/180*Math.PI, 10/180*Math.PI);
//         
//         PrintTools.info(""+node1.isValidNode());
//         ThreadTools.sleep(1000);
//         PrintTools.info(""+node2.isValidNode());
//         ThreadTools.sleep(1000);
//         PrintTools.info(""+node3.isValidNode());
//         ThreadTools.sleep(1000);
         
         
         
         RRTNode1DTimeDomain nodeRoot = new RRTNode1DTimeDomain(0.0, 0.0);
         
         rrtPlanner = new RRTPlannerSolarPanelCleaning(nodeRoot, cleaningPath);
         
         rrtPlanner.expandingTreesAndShortCut(200);
         
//         PrintTools.info("END shortcutting "+RRTNode3DTimeDomain.nodeValidityTester.numberOfTest);

         // *** message *** //
         PrintTools.info("Putting on Message");
         //motionFactory.setCleaningPath(cleaningPath);         
         //motionFactory.setMessage(rrtPlanner.getRRTPath());
         
         //wholeBodyTrajectoryMessage = motionFactory.getWholeBodyTrajectoryMessage();
         PrintTools.info("Complete putting on message");
         
         break;
         
         
      default:
         PrintTools.info("setTrajectoryMessage -> NONE");
         
         break;
         
      }
      
      
      
      return true;
   }

   
   
   
   
   
   
}
