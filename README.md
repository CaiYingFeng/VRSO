# VRSO

**VRSO**: **V**isual-Centric **R**econstruction for **S**tatic **O**bject Annotation, waymo example:  
<p align="left">
  <img src="assets/teaser.png" width="80%"/>
</p>
Comparison between our proposed VRSO (green) and Waymo (red) annotations after reprojection (from 3D space to 2D images). All images are from the Waymo Open Dataset (WOD). We can easily observe the reprojection errors (false positives and false negatives) among the Waymo annotations. For instance, the traffic signs (in both full and zoomed regions) are ignored or wrongly labelled in (a) and (c), while the red boxes do not tightly cover the targets in (b) and (d). Differently, VRSO yields consistent and accurate annotation among all images, even in low-resolution and illuminance conditions (b).

## Pipeline
<p align="left">
  <img src="assets/pipeline.png" width="80%"/>
  <img src="assets/process.png" width="80%"/>
</p>
