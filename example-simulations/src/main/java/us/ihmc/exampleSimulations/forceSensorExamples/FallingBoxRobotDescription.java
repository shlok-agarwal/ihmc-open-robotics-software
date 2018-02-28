package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class FallingBoxRobotDescription extends RobotDescription
{
   public FallingBoxRobotDescription(String name, Box3D box, double mass, boolean useGroundContactPoints)
   {
      super(name);

      FloatingJointDescription bodyJoint = new FloatingJointDescription("body", "bodyjointvariablename");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(mass, box.getLength(), box.getWidth(), box.getHeight());

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -0.5 * box.getHeight());
      bodyLinkGraphics.addCube(box.getLength(), box.getWidth(), box.getHeight(), YoAppearance.Red());
      bodyLinkGraphics.identity();
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      bodyJoint.setLink(bodyLink);

      if (useGroundContactPoints)
      {
         int idOfGCP = 0;
         GroundContactPointDescription gcpDescription1 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), 0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription1);
         idOfGCP++;

         GroundContactPointDescription gcpDescription2 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), 0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription2);
         idOfGCP++;

         GroundContactPointDescription gcpDescription3 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), -0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription3);
         idOfGCP++;

         GroundContactPointDescription gcpDescription4 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), -0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription4);
         idOfGCP++;

         GroundContactPointDescription gcpDescription5 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), 0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription5);
         idOfGCP++;

         GroundContactPointDescription gcpDescription6 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), 0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription6);
         idOfGCP++;

         GroundContactPointDescription gcpDescription7 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), -0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription7);
         idOfGCP++;

         GroundContactPointDescription gcpDescription8 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), -0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription8);
         idOfGCP++;
      }

      this.addRootJoint(bodyJoint);

   }

}
