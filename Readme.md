# Installation

---

The following should be the easiest way to install and run this project. 

1. Clone this repo and the related [Unity Assets package](https://gits-15.sys.kth.se/DD2438/MASUnityAssets)
2. Place them in a \<parent\> directory such that the repos are sibling folders. I.e.
   * \<parent\>/MASUnityAssets 
   * \<parent\>/2-Assingment-MAS2026
3. Run the assignment project as normal.

## Package resolution failed?

---

Relative path configuration failures are relatively common. If this occurs, you can either

1. Edit the path in the packages file.
   * Open the package file in this repository. "./Packages/manifest.json".
   * Edit the relative path for "com.mas.assets" to an [absolute path](https://docs.unity3d.com/Manual/upm-localpath.html).
2. Or re-install the package from the Unity Editor.
   * Open the package manager in the editor. 
   * Remove the package.
   * Install the pack!age from disk.

![Open the package manager by navigating the application top-menu to Window/Package Manager](/Images/OpenManager.png "Open package manager by navigating the application top-menu to Window/Package Manager")

![To remove packages, ensure the Packages: filter is set to InProject. Select the MAS Assets package and click Remove on the right hand side panel that appears. ](/Images/RemovePackage.png "To remove packages, ensure the Packages: filter is set to InProject. Select the MAS Assets package and click Remove on the right hand side panel that appears.")

![Install package from disk by clicking on the plus sign and choosing: Add package from disk...](/Images/AddFromDisk.png "Install package from disk by clicking on the plus sign and choosing: Add package from disk...")
