# Installation
This library can be installed dircetly from this git repository.
```bash
pip install git+https://github.com/HB0N0/fieldpath.git
```

# Basic Workflow
This library is intended for basic coverage path planning of autonomus agricultural vehicles. Generating a path requires 3 steps:
1. Defining the **field geometry** (and obstacles)
2. Generate a suitable **pattern** of unordered working paths which should cover the whole field area
3. Order the generated path segements to have only short **traversal paths** inbetween them. While ordering generate traversal paths, that connect the path working path segments.

After following these steps we get a ordered list of waypoints to follow which can be used in any navigation software (i.E. Nav2)

# First Steps
The steps here are to explain the basic usage. More advanced options like polygon decomposition and turn radius are possible and explained in other chapters.
## 1. Defining a field geometry

Field geometries are handled as [Shaply Polygon](https://shapely.readthedocs.io/en/stable/reference/shapely.Polygon.html) Objects. Have a look at the documentation to have a better understanding on how to generate them.

> [!Note] 
> Shapely Polygons may also have holes, which is supported by this library.

Generating a basic square (10x10) field polygon may look like:
```python
from shapely import Polygon

field_outline = ((0,0), (0,10), (10,10), (10,0), (0,0))
my_field = Polygon(field_outline)
```
## 2. Generate working paths
Next step is to generate a set of working paths in order to cover the field. This is done using the _FieldPathPlanner_ Class in the planner module. Parameters for class initilaisation are **working_width**, **turn_radius** and **traversal_offset**. For basic path generation only the working with is required.

```python
from fieldpath.planner import FieldPathPlanner

# Initialize class
fp = FieldPathPlanner(working_width = 0.3)
```
For generating paths currently 3 different patterns are possible:
### A-B Line Pattern
Pattern of alternating straight lines

```python
    pattern_angle = 90 #deg
    working_paths = fp.generate_A_B_line(my_field, pattern_angle)
    # working_paths is an shapely MultiLineString
```

### Contour Line Pattern
Pattern that follows the field outlines
```python
    num_contours = 3
    working_paths, _ = fp.generate_contour_paths(my_field, num_contours)
    # working_paths is an shapely MultiLineString
```
### Contoured A-B Lines (Recommended)
To generate driving paths that are within the field outlines, so the vehicle keeps inside all the time it is an option to drive a A-B Pattern inside the field, which is surrounded by some contour paths, which will cover the turning paths required to drive the A-B pattern.
```python
num_contours = 2
working_paths, _ = fp.generate_contoured_a_b_paths(my_field, num_contours)
# working_paths is an shapely MultiLineString
```
>[!TIP]
>For most usecases simply use this function. To get an simple A-B Line pattern the _num_contours_ variable can be set to `0`. This fuction handles some special cases automatically, which is not implemented in the other two pattern generation functions yet.

## 3. Generate driving sequence and traversal paths

Next step is to generate a driving sequence of the working paths. For that we need a start position and an optional end position. If not set the generated path will end on the waypoint of the last working path. 

```python
start_pose = (0, 0, 0)

coverage_path, path_types = fp.generate_work_sequence(working_paths,start_pose, field = my_field)

# coverage_path is an shapely MultiLineString.
# path_types can be used to distinguish between working and traversal path segments and is an ordered list of strings
```

Congratulations you have done your first coverage path planning :D