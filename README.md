# VRSO
Thank you for visiting our project repository. We're excited to announce that our code will be open-sourced soon!

**VRSO**: **V**isual-Centric **R**econstruction for **S**tatic **O**bject Annotation, waymo example:  
<p align="left">
  <img src="assets/teaser.png" width="80%"/>
</p>
The reprojection of 3D annotation in image space. Reprojected images are zoomed in to show a detailed comparison of annotation
from our method (in green boxes) and Waymo Open Dataset (WOD) manual labeling (in red boxes). It can be seen that the annotation
of WOD is not always consistent with the 2D images. It also contains some false positives (FP) and false negatives (FN) results. For
example, there are signs in the image but annotation from WOD did not show positive results, (a, c). The reprojection annotation in
(b) shows that the signboard is not visible in the image. In contrast, VRSO can provide more accurate and consistent 3D annotation in
different cases even in poor image quality and difficult illuminance conditions(d, e, f)

## Pipeline
<p align="left">
  <img src="assets/pipeline.png" width="80%"/>
  <img src="assets/process.png" width="80%"/>
</p>
