# MagicalHands: A Prototype Tool for Animation Authoring using Mid-Air Hand Gestures in VR

This code implements the animation tool described in 

"Rahul Arora, Rubaiat Habib Kazi, Danny Kaufman, Wilmot Li, and Karan Singh. 2019. MagicalHands: Mid-Air Hand Gestures for Animating in VR. In Proceedings of the 32nd Annual ACM Symposium on User Interface Software and Technology (UIST '19). ACM, New York, NY, USA. http://dx.doi.org/10.1145/3332165.3347942."

## Installation and Usage Instructions (Binary)
1. Download from `https:///github.com/rarora7777/MagicalHands/releases`
2. Extract and run on PowerShell or classic `cmd` console (tested on Windows 7 and 10):

   `./MagicalHands.exe --File ./Results/loop_de_loop.json`
   
   to load the result `loop_de_loop` from Fig. 1(e--f). Please see `./Results` for other results from the paper.
   
   Forgo the `--File` option to start creating your own animations:
   
   `./MagicalHands.exe`
   
**Note:** You can also use Windows Explorer GUI instead of command line, but passing arguments to load a saved scene is painful.

## Installation and Usage Instructions (Source)
1. Start with cloning the repo, insluding submodules

   `git clone --recursive https://github.com/rarora7777/MagicalHands`

   If you have already cloned the repo without submodules, then use

   `git submodule update --init --recursive`

2. Open the root folder in Unity editor, and download the Oculus Integration Plugin from the Asset Store (within Unity editor)
   https://assetstore.unity.com/packages/tools/integration/oculus-integration-82022. Import everything.

3. We're using a custom version of `/Oculus/Avatar/Scripts/OvrAvatar.cs` to allow inheritence. So, downloading the Oculus    Integration plugin replaces this custom file. Close Unity, and use

   `git checkout -- ./Assets/Oculus/Avatar/Scripts/OvrAvatar.cs`

   `git checkout -- ./Assets/Oculus/Avatar/Scripts/OvrAvatar.cs.meta`

   **AFTER** downloading the plugin to restore our customized version of the script.

   Alternatively, you can use `git reset --hard`, but make sure you know what you're doing.
   
4. Re-open the project in Unity and navigate to the scene `./Assets/Scenes/Main.Unity`. Now you're ready to run and tinker with the code.

*Other dependencies*

1. Oculus Rift Runtime: https://www.oculus.com/setup/

2. Unity: https://store.unity.com/ (Tested with version 2018.3.6f1&ndash;2018.3.8f1)

*Hardware*

Oculus Rift with Touch controllers

*Settings in Unity*

1. Ensure that VR SDKs are enabled (should be automatic): https://docs.unity3d.com/Manual/VROverview.html

2. Ensure that the Oculus package is installed (should be automatic): Window > Package Manager > In Project should show "Oculus (Desktop)" v1.29 installed.

<h2>Running the program</h2>

Load the scene `Assets/Scences/Main.unity` in the Unity interface and run.

<h2>Keyboard Controls</h2>

`Ctrl+S` Save animated scene

`Ctrl+E` Change environment map (cycles between a preset list)

`Ctrl+H` Hide ground plane

`Ctrl+Z` Undo last command (Also available via 3DUI)

**Note:** When running within Unity Editor, skip the `Ctrl` key for all the above. `Ctrl` is only needed when running the executable.

## Saving and Loading scenes

Scenes are stored as `JSON` files.

&mdash; To load a scene, use the `--File` option when running the executable.

&mdash; When saving a scene, a file name is chosen randomly; you cannot specify it. However, you can choose the save loction (folder). This can be done either using the `--SaveFolder` option when runnning the executable, or by editing `./StreamingAssets/save_folder.txt`. If both the command line option and the file are present, then the command line option is given preference. If neither is specified, scens are saved to the folder containing the executable.

## Citing
If this code is useful to you, please cite

```
@inproceedings{Arora:2019:MagicalHands,
 author = {Arora, Rahul and Kazi, Rubaiat Habib and Kaufman, Danny and Li, Wilmot and Singh, Karan},
 title = {MagicalHands: Mid-Air Hand Gestures for Animating in VR},
 booktitle = {Proceedings of the 32nd Annual ACM Symposium on User Interface Software and Technology},
 series = {UIST '19},
 year = {2019},
 location = {New Orleans, LA, USA},
 numpages = {15},
 publisher = {ACM},
 address = {New York, NY, USA},
 keywords = {gestures, virtual reality, animation},
} 
```
