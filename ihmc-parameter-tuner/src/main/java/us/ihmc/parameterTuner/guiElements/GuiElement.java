package us.ihmc.parameterTuner.guiElements;

public abstract class GuiElement
{
   public static final String SEPERATOR = ":";

   private final String name;
   private final GuiRegistry parent;
   private final String uniqueName;

   public GuiElement(String name, GuiRegistry parent)
   {
      this(name, parent, createUniqueName(name, parent));
   }

   public GuiElement(String name, GuiRegistry parent, String uniqueName)
   {
      if (name.contains(SEPERATOR))
      {
         throw new RuntimeException("Name " + name + " should never contain " + SEPERATOR);
      }

      this.name = name;
      this.parent = parent;
      this.uniqueName = uniqueName;
   }

   public String getName()
   {
      return name;
   }

   public String getUniqueName()
   {
      return uniqueName;
   }

   public GuiRegistry getParent()
   {
      return parent;
   }

   public static String createUniqueName(String name, GuiRegistry parent)
   {
      if (parent == null)
      {
         return name;
      }
      else
      {
         return parent.getUniqueName() + SEPERATOR + name;
      }
   }
}
