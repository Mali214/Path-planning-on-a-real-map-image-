# Path-planning-on-a-real-map-image-
This subsection will be discussing the performance of the different planning algorithms will be
compared as well as their properties. A comparison criterion will be presented and testing
environment (MSA university campus) preparation will be shown.

Preparation of the testing map

Before proceeding with testing and comparison between the different algorithms discussed in
earlier there must be an environment (map) prepared for these algorithms. to take a more practical
approach for testing a satellite image of a university campus was chosen to be the planning
environment shown in Figure 4.1.
The image used is not well used to be used directly to be used in a MATLAB simulation for path
planning so some image processing was done to obtain a more usable image. The goal here was to
differentiate between the drivable surface (roads) and the non-drivable surface (curbs, building,
grass, etc.). A standard procedure here is to threshold the image. However, because the roads,
buildings, and curbs have the same range of grayscale the resulting image will lose a lot of
important details, see Figure 4.1, the satellite image has to be tuned before thresholding. To tune
the satellite image, the image was sharpened then the contrast level was increased, a median filter
was applied to smooth out the image to produce the tuned image in Figure 4.3. Finally, the image
was the threshold to mark the drivable surface by 1 and the non-drivable surface by 0 as shown in
Figure 4.4.

![image](https://user-images.githubusercontent.com/122736585/212552391-13d38c32-a76a-4f8f-8db3-6700f7fe00fc.png)

