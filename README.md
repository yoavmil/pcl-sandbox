# pcl-sandbox
As the name implies, this repo is for playing with `pcl`.

## prism
The `pcl::ExtractPolygonalPrismData` is meant for filtering all the points that are inside/outside a prism, constructed from a plane & hull. 
Ex. the flow is:
* Create a random cloud with some planar data in it
* Find the plane in the cloud
* Find the concave hull of that planar data
* Construct a prism filter based on the hull

I have investigate a bug, found it and fixed it. The bug was the prism didn't catch correctly the points in and out of the prism when the prism was made of a multi-polygon with holes.

That happened because the algorithm used to determine if a point is in a polygon, didn't take in account that the hull could be of multiple polygons, and checked also against edges linking between the polygons, that didn't exist in the polygons themselves.
