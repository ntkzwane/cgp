//
// Mesh
//

#include "mesh.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <list>
#include <sys/stat.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/intersect.hpp>
#include <unordered_map>

using namespace std;
using namespace cgp;

GLfloat stdCol[] = {0.7f, 0.7f, 0.75f, 0.4f};
const int raysamples = 2;

void Sphere::genGeometry(ShapeGeometry * geom, View * view)
{
    glm::mat4 tfm, idt;
    glm::vec3 trs;

    idt = glm::mat4(1.0f); // identity matrix
    trs = glm::vec3(c.x, c.y, c.z);
    tfm = glm::translate(idt, trs);
    geom->genSphere(r, 40, 40, tfm);
}

bool Sphere::pointContainment(cgp::Point pnt)
{
    cgp::Vector distance;
    distance.diff(c,pnt);
    float distSqrd = distance.sqrdlength();
    if(distSqrd < r*r){
        return true;
    }else{return false;}
}

void Cylinder::genGeometry(ShapeGeometry * geom, View * view)
{
    glm::mat4 tfm, idt;
    glm::vec3 trs, rot;
    Vector edgevec, zerovec, axisvec, zaxis;
    float edgelen, aval;

    edgevec.diff(s, e);
    edgelen = edgevec.length();
    edgevec.normalize();

    // translate to starting vertex
    trs = glm::vec3(s.x, s.y, s.z);
    tfm = glm::translate(idt, trs);

    // determine rotation axis, normal to plane of z axis and edge vector
    zaxis = Vector(0.0f, 0.0f, 1.0f);
    axisvec.cross(zaxis, edgevec);

    // check for degenerate cases
    if(axisvec == zerovec)
    {
        if(!(edgevec == zaxis)) // diametrically opposite
        {
            aval = 180.0f;
            rot = glm::vec3(0.0f, 1.0f, 0.0f);
            tfm = glm::rotate(tfm, aval, rot);
        }
        // otherwise aligned so no rotation
    }
    else
    {
        axisvec.normalize();
        aval = acosf(zaxis.dot(edgevec)) * RAD2DEG;
        rot = glm::vec3(axisvec.i, axisvec.j, axisvec.k);
        tfm = glm::rotate(tfm, aval, rot);
    }

    // align to edge vector by rotation
    geom->genCylinder(r, edgelen, 12, 4, tfm);
}

bool Cylinder::pointContainment(cgp::Point pnt)
{
    cgp::Vector dirvec;
    float dist, tval;

    // find distance and parameter value to closest point on the axis of the cylinder
    dirvec.diff(s, e);
    rayPointDist(s, dirvec, pnt, tval, dist);
    if(tval >= 0.0f && tval <= 1.0f && dist <= r)
        return true;
    else
        return false;
}

bool Mesh::findVert(cgp::Point pnt, int &idx)
{
    bool found = false;
    int i = 0;

    idx = -1;
    // linear search of vertex list
    while(!found && i < (int) verts.size())
    {
        if(verts[i] == pnt)
        {
            found = true;
            idx = i;
        }
        i++;
    }
    return found;
}

long Mesh::hashVert(cgp::Point pnt, cgp::BoundBox bbox)
{
    long x, y, z;
    float range = 2500.0f;
    long lrangesq, lrange = 2500;

    lrangesq = lrange * lrange;

    // discretise vertex within bounds of the enclosing bounding box
    x = (long) (((pnt.x - bbox.min.x) * range) / bbox.diagLen()) * lrangesq;
    y = (long) (((pnt.y - bbox.min.y) * range) / bbox.diagLen()) * lrange;
    z = (long) (((pnt.z - bbox.min.z) * range) / bbox.diagLen());
    return x+y+z;
}

void Mesh::mergeVerts()
{
    vector<cgp::Point> cleanverts;
    long key;
    int i, p, hitcount = 0;
    // use hashmap to quickly look up vertices with the same coordinates
    std::unordered_map<long, int> idxlookup; // key is concatenation of vertex position, value is index into the cleanverts vector
    cgp::BoundBox bbox;

    // construct a bounding box enclosing all vertices
    for(i = 0; i < (int) verts.size(); i++)
        bbox.includePnt(verts[i]);

    // remove duplicate vertices
    for(i = 0; i < (int) verts.size(); i++)
    {
        key = hashVert(verts[i], bbox);
        if(idxlookup.find(key) == idxlookup.end()) // key not in map
        {
            idxlookup[key] = (int) cleanverts.size(); // put index in map for quick lookup
            cleanverts.push_back(verts[i]);
        }
        else
        {
            hitcount++;
        }
    }
    cerr << "num duplicate vertices found = " << hitcount << " of " << (int) verts.size() << endl;
    cerr << "clean verts = " << (int) cleanverts.size() << endl;
    cerr << "bbox min = " << bbox.min.x << ", " << bbox.min.y << ", " << bbox.min.z << endl;
    cerr << "bbox max = " << bbox.max.x << ", " << bbox.max.y << ", " << bbox.max.z << endl;
    cerr << "bbox diag = " << bbox.diagLen() << endl;


    // re-index triangles
    for(i = 0; i < (int) tris.size(); i++)
        for(p = 0; p < 3; p++)
        {
            key = hashVert(verts[tris[i].v[p]], bbox);
            if(idxlookup.find(key) != idxlookup.end())
                tris[i].v[p] = idxlookup[key];
            else
                cerr << "Error Mesh::mergeVerts: vertex not found in map" << endl;

        }

    verts.clear();
    verts = cleanverts;
}

void Mesh::deriveVertNorms()
{
    vector<int> vinc; // number of faces incident on vertex
    int p, t;
    cgp::Vector n;

    // init structures
    for(p = 0; p < (int) verts.size(); p++)
    {
        vinc.push_back(0);
        norms.push_back(cgp::Vector(0.0f, 0.0f, 0.0f));
    }

    // accumulate face normals into vertex normals
    for(t = 0; t < (int) tris.size(); t++)
    {
        n = tris[t].n; n.normalize();
        for(p = 0; p < 3; p++)
        {
            norms[tris[t].v[p]].add(n);
            vinc[tris[t].v[p]]++;
        }
    }

    // complete average
    for(p = 0; p < (int) verts.size(); p++)
    {
        norms[p].mult(1.0f/((float) vinc[p]));
        norms[p].normalize();
    }
}

void Mesh::deriveFaceNorms()
{
    int t;
    cgp::Vector evec[2];

    for(t = 0; t < (int) tris.size(); t++)
    {
        // right-hand rule for calculating normals, i.e. counter-clockwise winding from front on vertices
        evec[0].diff(verts[tris[t].v[0]], verts[tris[t].v[1]]);
        evec[1].diff(verts[tris[t].v[0]], verts[tris[t].v[2]]);
        evec[0].normalize();
        evec[1].normalize();
        tris[t].n.cross(evec[0], evec[1]);
        tris[t].n.normalize();
    }

}

void Mesh::buildTransform(glm::mat4x4 &tfm)
{
    glm::mat4x4 idt;

    idt = glm::mat4(1.0f);
    tfm = glm::translate(idt, glm::vec3(trx.i, trx.j, trx.k));
    tfm = glm::rotate(tfm, zrot, glm::vec3(0.0f, 0.0f, 1.0f));
    tfm = glm::rotate(tfm, yrot, glm::vec3(0.0f, 1.0f, 0.0f));
    tfm = glm::rotate(tfm, xrot, glm::vec3(1.0f, 0.0f, 0.0f));
    tfm = glm::scale(tfm, glm::vec3(scale));
}

void Mesh::buildSphereAccel(int maxspheres)
{
    vector<Sphere> packedspheres;
    list<int> insphere;
    int i, v, p, t, s, numsx, numsy, numsz, sx, sy, sz, x, y, z, sind = 0;
    cgp::Point centroid, center;
    cgp::Vector radvec, diag, edgevec;
    float radius, dist, packrad, packspace, maxedge;
    cgp::BoundBox bbox;
    Sphere sph;
    bool found;

    if((int) tris.size() > 0) // must actually have a mesh to build the acceleration structure on top of
    {
        // calculate longest triangle edge
        maxedge = 0.0f;
        for(t = 0; t < (int) tris.size(); t++)
        {
            edgevec.diff(verts[tris[t].v[0]], verts[tris[t].v[1]]);
            dist = edgevec.length();

            edgevec.diff(verts[tris[t].v[1]], verts[tris[t].v[2]]);
            dist = min(dist, edgevec.length());

            edgevec.diff(verts[tris[t].v[2]], verts[tris[t].v[0]]);
            dist = min(dist, edgevec.length());
            if(dist > maxedge)
                maxedge = dist;
        }

        // calculate mesh bounding box
        for(v = 0; v < (int) verts.size(); v++)
            bbox.includePnt(verts[v]);

        diag = bbox.getDiag();
        dist = std::max(diag.i, diag.j); dist = std::max(dist, diag.k);
        packrad = dist / (float) maxspheres;
        if(packrad < maxedge) // sphere radius must be larger than the longest shorest edge in any triangle
        {
            cerr << "Error Mesh::buildSphereAccel: bounding spheres too small relative to edge length. Inflating spheres accordingly." << endl;
            packrad = maxedge;
        }
        packspace = 2.0f * packrad;

        // number of spheres to pack in each dimension filling the bounding box
        numsx = (int) ceil(diag.i / packspace) +1;
        numsy = (int) ceil(diag.j / packrad) +1;
        numsz = (int) ceil(diag.k / packrad) +1;

        // pack spheres into volume so that there is no unsampled space
        // alternating rows are offset by the sphere radius and x-adjacent spheres are spaced to just touch
        // this provides a minimal overlap between spheres that nevertheless covers all available space
        for(x = 0; x < numsx; x++)
            for(y = 0; y < numsy; y++)
                for(z = 0; z < numsz; z++)
                {
                    center = cgp::Point((float) x * packspace + bbox.min.x, (float) y * packrad + bbox.min.y, (float) z * packrad + bbox.min.z);
                    if(z%2==1) // row shifting alternates in layers
                    {
                        if(y%2 == 1) // shift alternating rows by half spacing in x
                            center.x += packrad;
                    }
                    else
                    {
                        if(y%2 == 0) // shift alternating rows by half spacing in x
                            center.x += packrad;
                    }

                    sph.c = center;
                    sph.r = packrad;
                    sph.ind.clear();
                    packedspheres.push_back(sph);
                }

        // assign triangles to spheres based on their vertices
        for(t = 0; t < (int) tris.size(); t++)
        {

            // find the first sphere that contains any of the triangle vertices
            found = false; s = 0;
            while(!found && s < (int) packedspheres.size())
            {
                for(p = 0; p < 3; p++)
                    if(packedspheres[s].pointContainment(verts[tris[t].v[p]]))
                    {
                        found = true;
                        // add triangle to packedsphere
                        packedspheres[s].ind.push_back(t);
                        sind = s;
                    }
                s++;
            }

            if(!found)
            {
                cerr << "Error Mesh::buildSphereAccel: packed spheres do not entirely fill the volume." << endl;
            }
            else
            {
                // once a sphere triangle intersection is found only a few adjacent spheres need be tested for intersection
                // also test against +x, +y, +z spheres (and diagonal adjacents as well for possible intersection due to partial sphere overlaps

                // neighbour search of packed spheres
                // decompose into x, y, z sphere grid position

                // unflatten index
                sx = (int) ((float) sind / (float) (numsy*numsz));
                sy = (int) ((float) sind / (float) numsz);
                sz = sind - (sx * numsy * numsz + sy * numsz);
                for(x = sx; x <= sx+1; x++)
                    for(y = sy; y <= sy+1; y++)
                        for(z = sz; z <= sz+1; z++)
                        {
                            if(x < numsx && y < numsy && z < numsz) // bounds check
                                if(!(x == sx && y == sy && z == sz)) // ignore starter sphere
                                {
                                    s = x * numsy * numsz + y * numsz + z; // flatten 3d index
                                    found = false;
                                    for(p = 0; p < 3; p++)
                                        if(packedspheres[s].pointContainment(verts[tris[t].v[p]]))
                                            found = true;
                                    if(found) // add triangle to packedsphere
                                        packedspheres[s].ind.push_back(t);
                                }

                        }

            }
        }

        // discard any spheres containing no triangles
        for(i = 0; i < (int) packedspheres.size(); i++)
        {
            if(!packedspheres[i].ind.empty()) // bounding sphere has at least one triangle
                boundspheres.push_back(packedspheres[i]);
        }
        packedspheres.clear();

        // recaculate spheres by using center of mass and furthest triangle vertex to get tighter bounding
        for(i = 0; i < (int) boundspheres.size(); i++)
        {
            insphere.clear();
            // gather included vertices
            for(t = 0; t < (int) boundspheres[i].ind.size(); t++)
                for(p = 0; p < 3; p++)
                    insphere.push_back(tris[boundspheres[i].ind[t]].v[p]);

            // remove duplicates because they will bias the center of mass
            insphere.sort();
            insphere.unique();

            // calculate centroid as average of vertices
            centroid = cgp::Point(0.0f, 0.0f, 0.0f);
            for (list<int>::iterator it=insphere.begin(); it!=insphere.end(); ++it)
            {
                centroid.x += verts[(* it)].x; centroid.y += verts[(* it)].y; centroid.z += verts[(* it)].z;

            }
            centroid.x /= (float) insphere.size();  centroid.y /= (float) insphere.size();  centroid.z /= (float) insphere.size();
            boundspheres[i].c = centroid;

            // calculate new radius
            radius = 0.0f;
            for (list<int>::iterator it=insphere.begin(); it!=insphere.end(); ++it)
            {
                radvec.diff(boundspheres[i].c, verts[(* it)]);
                dist = radvec.length();
                if(dist > radius)
                    radius = dist;
            }
            boundspheres[i].r = radius;

            /*
             // check current radius
             for (list<int>::iterator it=insphere.begin(); it!=insphere.end(); ++it)
             {
             radvec.diff(boundspheres[i].c, verts[(* it)]);
             dist = radvec.length();
             if(dist > boundspheres[i].r)
             cerr << "Error Mesh::buildSphereAccel:vertex " << (* it) << " outside containing sphere by " << (dist - packrad) << endl;
             }*/
        }
    }
}

Mesh::Mesh()
{
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
}

Mesh::~Mesh()
{
    clear();
}

void Mesh::clear()
{
    verts.clear();
    tris.clear();
    geometry.clear();
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
}

void Mesh::genGeometry(ShapeGeometry * geom, View * view)
{
    vector<int> faces;
    int t, p;
    glm::mat4x4 tfm;

    // transform mesh data structures into a form suitable for rendering
    // by flattening the triangle list
    for(t = 0; t < (int) tris.size(); t++)
        for(p = 0; p < 3; p++)
            faces.push_back(tris[t].v[p]);

    // construct transformation matrix
    buildTransform(tfm);
    geom->genMesh(&verts, &norms, &faces, tfm);
}

bool Mesh::bindGeometry(View * view, ShapeDrawData &sdd)
{
    geometry.clear();
    geometry.setColour(col);
    genGeometry(&geometry, view);

    // bind geometry to buffers and return drawing parameters, if possible
    if(geometry.bindBuffers(view))
    {
        sdd = geometry.getDrawParameters();
        return true;
    }
    else
       return false;
}

bool Mesh::pointContainment(cgp::Point pnt)
{
    int incount = 0, outcount = 0, hits, i, t, p, s;
    glm::vec3 v[3], origin, xsect, ray;
    glm::mat4x4 tfm, idt;
    glm::vec4 vxfm, cxfm;
    cgp::Point vert, sphc;
    cgp::Vector dir;
    float dist, tval;
    list<int> inspheres;

    srand(time(0));

    // sample over multiple rays to avoid numerical issues (e.g., ray hits a vertex or edge)
    origin = glm::vec3(pnt.x, pnt.y, pnt.z);

    // construct transformation matrix
    buildTransform(tfm);

    if(boundspheres.empty()) // no acceleration structure so build
        buildSphereAccel((int) sphperdim);

    for(i = 0; i < raysamples; i++)
    {
        hits = 0;
        inspheres.clear();

        // sampling ray with random direction
        // avoid axis aligned rays because more likely to lead to numerical issues with axis aligned structures
        dir = cgp::Vector((float) (rand()%1000-500), (float) (rand()%1000-500), (float) (rand()%1000-500));
        dir.normalize();
        ray = glm::vec3(dir.i, dir.j, dir.k);

        // gather potential triangle intersector indices and remove duplicates
        for(s = 0; s < (int) boundspheres.size(); s++)
        {
            // sphere accel structure is in model space, so need to apply transform
            cxfm = tfm * glm::vec4(boundspheres[s].c.x, boundspheres[s].c.y, boundspheres[s].c.z, 1.0f);
            sphc = cgp::Point(cxfm.x, cxfm.y, cxfm.z);

            rayPointDist(pnt, dir, sphc, tval, dist);
            if(dist <= boundspheres[s].r) // if ray hits the bounding sphere
                for(t = 0; t < (int) boundspheres[s].ind.size(); t++) // include all triangles allocated to the bounding sphere
                    inspheres.push_back(boundspheres[s].ind[t]);
        }

        // remove duplicate triangle indices because a triangle may appear in multiple bounding spheres
        inspheres.sort();
        inspheres.unique();

        // test intersection against list of triangles
        for (list<int>::iterator it=inspheres.begin(); it!=inspheres.end(); ++it)
        {
            for(p = 0; p < 3; p++)
            {
                vert = verts[tris[(* it)].v[p]];
                vxfm = tfm * glm::vec4(vert.x, vert.y, vert.z, 1.0f);
                v[p] = glm::vec3(vxfm.x, vxfm.y, vxfm.z);
            }
            if(glm::intersectRayTriangle(origin, ray, v[0], v[1], v[2], xsect) || glm::intersectRayTriangle(origin, ray, v[0], v[2], v[1], xsect)) // test triangle in both windings because intersectLineTriangle is winding dependent
                hits++;
        }

        if(hits%2 == 0) // even number of intersection means point is outside
            outcount++;
        else // point is inside
            incount++;
    }
    
    // consensus wins
    return (incount > outcount);
}

void Mesh::boxFit(float sidelen)
{
    cgp::Point pnt;
    cgp::Vector shift, diag, halfdiag;
    float scale;
    int v;
    cgp::BoundBox bbox;

    // calculate current bounding box
    for(v = 0; v < (int) verts.size(); v++)
        bbox.includePnt(verts[v]);

    if((int) verts.size() > 0)
    {
        // calculate translation necessary to move center of bounding box to the origin
        diag = bbox.getDiag();
        shift.pntconvert(bbox.min);
        halfdiag = diag; halfdiag.mult(0.5f);
        shift.add(halfdiag);
        shift.mult(-1.0f);

        // scale so that largest side of bounding box fits sidelen
        scale = max(diag.i, diag.j); scale = max(scale, diag.k);
        if(scale > 0.0f)
        {
            scale = sidelen / scale;

            // shift center to origin and scale uniformly
            for(v = 0; v < (int) verts.size(); v++)
            {
                pnt = verts[v];
                shift.pntplusvec(pnt, &pnt);
                pnt.x *= scale; pnt.y *= scale; pnt.z *= scale;
                verts[v] = pnt;
            }
        }
        buildSphereAccel((int) sphperdim);
    }
}

bool Mesh::readSTL(string filename)
{
    FILE *f=fopen(filename.c_str(),"rb");
    if(!f) return 1;
    char title[80];
    unsigned int numFaces;
    fread(title,80,1,f);
    fread((void*)&numFaces,4,1,f);
    float v[12];
    unsigned short unit16;
    for(size_t i = 0; i < numFaces; i ++){
        for(size_t j = 0; j < 12; j++){
            fread((void*)&v[j],sizeof(float),1,f);
        }
        fread((void*)&unit16, sizeof(unsigned short), 1, f); // remove extra two bits
        // counstruct triangle normal and vertices
        cgp::Vector n(v[0],v[1],v[2]);
        cgp::Point v1(v[3],v[4],v[5]);
        cgp::Point v2(v[6],v[7],v[8]);
        cgp::Point v3(v[9],v[10],v[11]);
        // add the vertices to the list of vertices
        verts.push_back(v1);
        verts.push_back(v2);
        verts.push_back(v3);
        // construct the triangle
        Triangle t;
        t.n = n;
        t.v[0] = verts.size() - 3;
        t.v[1] = verts.size() - 2;
        t.v[2] = verts.size() - 1;
        
        norms.push_back(n);
        tris.push_back(t);
    }
    fclose(f);
cout << " --- number of faces -- " << numFaces << endl;
    // STL provides a triangle soup so merge vertices that are coincident
    /*mergeVerts();
    // normal vectors at vertices are needed for rendering so derive from incident faces
    deriveVertNorms();*/
    return true;
}

bool Mesh::writeSTL(string filename)
{
    // too early to release solution, since assignments can be submitted up to 3 days late
    return true;
}

bool Mesh::basicValidity()
{
    // too early to release solution, since assignments can be submitted up to 3 days late
    return true;
}

bool Mesh::manifoldValidity()
{
    // too early to release solution, since assignments can be submitted up to 3 days late
    return true;
}
