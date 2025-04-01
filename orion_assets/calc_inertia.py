#!/usr/bin/env python

"""
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>

From: https://github.com/gstavrinos/calc-inertia
"""
import sys
import stl
#(sudo pip install numpy-stl)

# Command line params:
# 1: STL file
# 2: Mass
# 3: Scale

def getDimensions(model):
    minx = maxx = miny = maxy = minz = maxz = None
    for p in model.points:
        if minx is None:
            minx = p[stl.Dimension.X]
            maxx = p[stl.Dimension.X]
            miny = p[stl.Dimension.Y]
            maxy = p[stl.Dimension.Y]
            minz = p[stl.Dimension.Z]
            maxz = p[stl.Dimension.Z]
        else:
            maxx = max(p[stl.Dimension.X], maxx)
            minx = min(p[stl.Dimension.X], minx)
            maxy = max(p[stl.Dimension.Y], maxy)
            miny = min(p[stl.Dimension.Y], miny)
            maxz = max(p[stl.Dimension.Z], maxz)
            minz = min(p[stl.Dimension.Z], minz)
    return maxx - minx, maxy - miny, maxz - minz


# Based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
def getInertia(x, y, z, m, s):
    xx = 1./12 * m * (y**2 + z**2) * s
    yy = 1./12 * m * (x**2 + z**2) * s
    zz = 1./12 * m * (x**2 + y**2) * s
    return xx, yy, zz

if __name__ == '__main__':
    model = stl.mesh.Mesh.from_file(sys.argv[1])
    m = float(sys.argv[2])
    scale = float(sys.argv[3])

    x, y, z = getDimensions(model)

    print("<inertia  ixx=\"%s\" ixy=\"0\" ixz=\"0\" iyy=\"%s\" iyz=\"0\" izz=\"%s\" />" % (getInertia(x, y, z, m, scale)))