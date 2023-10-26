# Mission planner

This module is kind of the main thing in this project. The Mission planner is responsible for finding good poses for inspecting the points of interest.

The calculation of the best possible inspection poses is carried out as follows:

- Points of interest are defined
- We find movement restrictions based on walkways (the inspector robot is not allowed outside walkways)
- We define possible inspection points for each x meters of the walkways
- For each point of interest, we loop through all the possible inspection points. For each possible inspection point, we calculate a score (meaning how well this point fits as an inspection point for the given POI)

The score of a possible inspection point is calculated based on the following:
- Number of faces between the point and the POI (meaning: can we see the POI?)
- Distance between point and POI
- Angle between point and defined inspection direction of the POI

## PyMesh

This module uses PyMesh. We compile PyMesh from source. Be aware that you need approximately 12GB of RAM to compile PyMesh.