package us.ihmc.robotics.parameterGui.tree;

import java.util.List;

import javafx.scene.control.TreeView;
import us.ihmc.robotics.parameterGui.RegularExpression;
import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTree extends TreeView<ParameterTreeValue>
{
   public ParameterTree()
   {
      super();
      setCellFactory(param -> new ParameterTreeCell());
   }

   public void setRegistries(List<Registry> registries, boolean hideNamespaces, String regex)
   {
      ParameterTreeItem root = new ParameterTreeItem(null);
      root.setExpanded(true);
      setShowRoot(false);
      setRoot(root);

      boolean searching = regex != null && !regex.isEmpty();
      if (hideNamespaces && searching)
      {
         addMatchingParametersRecursive(registries, root, regex);
      }
      else if (hideNamespaces)
      {
         addParametersRecursive(registries, root);
      }
      else if (searching)
      {
         addMatchingRecursive(registries, root, regex);
         sortChildren(root);
      }
      else
      {
         addRecursive(registries, root);
      }

      root.expandChildrenIfEmpty();
   }

   private static void sortChildren(ParameterTreeItem item)
   {
      item.getChildren().sort((o1, o2) -> {
         if (o1.getValue().isRegistry() && !o2.getValue().isRegistry())
         {
            return -1;
         }
         if (o2.getValue().isRegistry() && !o1.getValue().isRegistry())
         {
            return 1;
         }
         return 0;
      });
   }

   private static void addMatchingRecursive(List<Registry> registries, ParameterTreeItem item, String regex)
   {
      if (registries == null)
      {
         return;
      }
      registries.stream().forEach(registry -> {
         if (RegularExpression.check(registry.getName(), regex))
         {
            ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
            item.getChildren().add(registryItem);
            addRecursive(registry.getRegistries(), registryItem);
            addAll(registry.getParameters(), registryItem);
         }
         addAllMatching(registry.getParameters(), item, regex);
         addMatchingRecursive(registry.getRegistries(), item, regex);
      });
   }

   private static void addRecursive(List<Registry> registries, ParameterTreeItem item)
   {
      if (registries == null)
      {
         return;
      }
      registries.stream().forEach(registry -> {
         ParameterTreeItem registryItem = new ParameterTreeItem(new ParameterTreeRegistry(registry));
         item.getChildren().add(registryItem);
         addRecursive(registry.getRegistries(), registryItem);
         addAll(registry.getParameters(), registryItem);
      });
   }

   private static void addMatchingParametersRecursive(List<Registry> registries, ParameterTreeItem item, String regex)
   {
      if (registries == null)
      {
         return;
      }
      registries.stream().forEach(registry -> {
         addAllMatching(registry.getParameters(), item, regex);
         addMatchingParametersRecursive(registry.getRegistries(), item, regex);
      });
   }

   private static void addParametersRecursive(List<Registry> registries, ParameterTreeItem item)
   {
      if (registries == null)
      {
         return;
      }
      registries.stream().forEach(registry -> {
         addAll(registry.getParameters(), item);
         addParametersRecursive(registry.getRegistries(), item);
      });
   }

   private static void addAllMatching(List<Parameter> parameters, ParameterTreeItem item, String regex)
   {
      if (parameters == null)
      {
         return;
      }
      parameters.stream().filter(parameter -> RegularExpression.check(parameter.getName(), regex))
                .forEach(parameter -> item.getChildren().add(new ParameterTreeItem(new ParameterTreeParameter(parameter))));
   }

   private static void addAll(List<Parameter> parameters, ParameterTreeItem item)
   {
      if (parameters == null)
      {
         return;
      }
      parameters.stream().forEach(parameter -> item.getChildren().add(new ParameterTreeItem(new ParameterTreeParameter(parameter))));
   }
}
