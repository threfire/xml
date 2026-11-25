# MuJoCo export - 总装配提

## Files
- `xiaochuanleg_fixed.xml`: fixed base version.
- `xiaochuanleg_floating.xml`: free base with ground plane.
- `meshes/`: visual mesh assets referenced by the XML files.
- `*.swmechanism.xml`: source mechanism exports for traceability.
- `coordinate_axes_preview.png`: initial pose and base-axis view.

Keep the XML files and `meshes` directory together. The XMLs refer to mesh assets by relative path.

## View collision geometry
The MuJoCo viewer starts with geom groups 0, 1, and 2 visible. Visual meshes are group 0, collision shapes are group 1, and the base-axis bars plus floating ground are group 2. After opening, press `0` to hide/show visual meshes and `1` to hide/show collision geometry. Press `0` to hide the meshes and inspect the collision shapes clearly.

## Coordinate frame
The base frame is right-handed: +X forward, +Y left, +Z up. Red, green, and blue bars above the chassis show +X, +Y, +Z. The floating XML starts above the ground without initial penetration.

## Collision and linkage notes
- The base has four hip-axis hinges: left thigh, left rocker, right thigh, and right rocker. On each side, the thigh and rocker axes are coaxial. There is no extra `right_hip` joint.
- The linkage has eight `connect` equality constraints.
- Left collision geometry was retained unchanged. The right leg's 10 collision geoms mirror the corresponding left collision shapes in the right links' local frames.
- The six broad body exclusions for thigh/short-link, calf/mid-link, and mid-link/rocker were replaced by eight geom-pair contacts. Contact force thresholds use the earlier pair-specific margins of -2.3 mm, -2.1 mm, -2.7 mm, and -4.0 mm on each side. The collision box dimensions are unchanged.
- Collision geometry comprises 19 boxes and 6 cylinders. Visual meshes do not collide. The misplaced `滚珠及保持架` visuals were removed (9 geoms); the aggregate base-link mass/inertia was retained.

## Open in MuJoCo native viewer
Run in Windows Terminal or PowerShell (Python and `mujoco` installed):

```powershell
python -m mujoco.viewer --mjcf "C:\Users\CF\Desktop\小串腿\第二版\总装配提_description_四条基座共轴铰接_方盒碰撞\xiaochuanleg_fixed.xml"
python -m mujoco.viewer --mjcf "C:\Users\CF\Desktop\小串腿\第二版\总装配提_description_四条基座共轴铰接_方盒碰撞\xiaochuanleg_floating.xml"
```

Both compile with MuJoCo 4.0.1 and completed 3-second smoke tests. The reported total mass is about 2.329246 kg.


