package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.TempPreallocatedList;

public class Polygon2DMessage extends Packet<Polygon2DMessage>
{
   public TempPreallocatedList<Point3D> vertices = new TempPreallocatedList<>(Point3D.class, Point3D::new, 100);

   public Polygon2DMessage()
   {
   }

   @Override
   public void set(Polygon2DMessage other)
   {
      MessageTools.copyData(other.vertices, vertices);
   }

   public TempPreallocatedList<Point3D> getVertices()
   {
      return vertices;
   }

   @Override
   public boolean epsilonEquals(Polygon2DMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(vertices, other.vertices, epsilon);
   }
}
