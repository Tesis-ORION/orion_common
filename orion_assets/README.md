# ORION Assets

## Overview

This directory contains the related files for the design of ORION robot.

## License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Author**: Daniel Felipe López Escobar.

The *orion_description* package has been tested under [ROS](https://www.ros.org/) Jazzy.

## Content

You can include the next information and files:

- **[3DPrintAssets:](/orion_assets/3DPrintAssets/)** It is a collection of assets in STL so you can pass the files to your slicer and start building the ORION robot.

- **[FreeCadAssets:](/orion_assets/FreeCadAssets/)** They are the source CAD of the project, so if you want to make modifications or take a look of the pieces you can do it freely by using *FreeCad 1.0.0*.

- **IgesAssets:** If you aren't a FreeCad user, and still want to explore the CADs, here you can find them in a format that can be used with *SolidWorks*, *Blender*, among others.

- **[LasserAssets:](/orion_assets/LasserAssets/)** Another option to create some parts of ORION is by using laser cuts on MDF, acrilic or similar materials. This only applies to bases and walls.

- **[calc_inertia.py:](/orion_assets/calc_inertia.py)** It is a file oriented to obtain the inertia of STL and URDFS by providing the object path, the scale and the mass.
