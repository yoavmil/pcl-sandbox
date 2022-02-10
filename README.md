# pcl-sandbox
As the name implies, this repo is for playing with `pcl`.

## prism
The `pcl::ExtractPolygonalPrismData` is meant for filtering all the points that are inside/outside a prism, constructed from a plane & hull. 
Ex. the flow is:
* Create a random cloud with some planar data in it
* Find the plane in the cloud
* Find the concave hull of that planar data
* Construct a prism filter based on the hull

The prism project was for trying to investigate a bug where the prism didn't work as expected with complex polygons. So far, it behaves ok.
