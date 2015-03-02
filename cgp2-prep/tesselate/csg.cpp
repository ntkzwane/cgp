//
// csg
//

#include "csg.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <limits>
#include <stack>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace std;
// using namespace cgp;

GLfloat defaultCol[] = {0.243f, 0.176f, 0.75f, 1.0f};

bool Scene::genVizRender(View * view, ShapeDrawData &sdd)
{
    std::vector<ShapeNode *> leaves;
    int i;

    geom.clear();
    geom.setColour(defaultCol);

    // TO DO HERE, traverse csg tree pushing leaf nodes (shapes) to leaves vector
    // note: this displays all the constituent shapes in the tree but doesn't apply any set operations to them
    // so it is purely a pre-visualization
    std::stack<SceneNode*> nodeStack;
    SceneNode* curRoot = csgroot;
    
    cout << " -- in tree traverse for Scene::genVizRender(..) \n";
    while(!nodeStack.empty() or curRoot != nullptr){
        ShapeNode* curNode;
        OpNode* curOp;
        try{
            curNode = dynamic_cast<ShapeNode*>(curRoot);
            curOp = dynamic_cast<OpNode*>(curRoot);
            // check if the current node exists, if not, there are no more left nodes
            if(curRoot != nullptr){
            
                nodeStack.push(curRoot);
                if(curOp != nullptr){curRoot = curOp->left;} ////----------///// check if this doesn't stop us from visiting each node
            }else{
                curRoot = nodeStack.top();
                nodeStack.pop();
                // if it is a ShapeNode, add to list of leaves, else continue traverse
                if(curNode != nullptr){leaves.push_back(curNode);
                }else if(curOp != nullptr){curRoot = curOp->right;} ////----------///// check if this doesn't stop us from visiting each node
            }
        }catch(exception &e){
            cerr << " at Scene::genVizRender - encoutered problem while casting" << endl;
        }
    }

    // traverse leaf shapes generating geometry
    for(i = 0; i < (int) leaves.size(); i++)
    {
        leaves[i]->shape->genGeometry(&geom, view);
    }

    // bind geometry to buffers and return drawing parameters, if possible
    if(geom.bindBuffers(view))
    {
        sdd = geom.getDrawParameters();
        return true;
    }
    else
        return false;
}

bool Scene::genVoxRender(View * view, ShapeDrawData &sdd)
{
    int x, y, z, xdim, ydim, zdim;
    glm::mat4 tfm, idt;
    glm::vec3 trs;
    cgp::Point pnt;

    geom.clear();
    geom.setColour(defaultCol);

    if(voxactive)
    {
        idt = glm::mat4(1.0f); // identity matrix

        vox.getDim(xdim, ydim, zdim);

        // place a sphere at filled voxels but subsample to avoid generating too many spheres
        for(x = 0; x < xdim; x+=10)
            for(y = 0; y < ydim; y+=10)
                for(z = 0; z < zdim; z+=10)
                {
                    if(vox.get(x, y, z))
                    {
                        pnt = vox.getVoxelPos(x, y, z); // convert from voxel space to world coordinates
                        trs = glm::vec3(pnt.x, pnt.y, pnt.z);
                        tfm = glm::translate(idt, trs);
                        geom.genSphere(voxsidelen * 5.0f, 3, 3, tfm);
                    }
                }

    }

    // bind geometry to buffers and return drawing parameters, if possible
    if(geom.bindBuffers(view))
    {
        sdd = geom.getDrawParameters();
        return true;
    }
    else
        return false;
}

Scene::Scene()
{
    csgroot = NULL;
    col = defaultCol;
    voldiag = cgp::Vector(20.0f, 20.0f, 20.0f);
    voxsidelen = 0.0f;
    voxactive = false;
}

Scene::~Scene()
{
    clear();
}

void Scene::treeClear(SceneNode* root){
    // cast the current "root" node as one of the two node types, using dynamic casting
    ShapeNode* curNode;
    OpNode* curOp;
    /*int opCounter = 0;
    int shapeCounter = 0;*/
    try{
        cout << " -- entering root node -- \n";
        curNode = dynamic_cast<ShapeNode*>(root);
        curOp = dynamic_cast<OpNode*>(root);

        if(root != nullptr){
            // if the current node is an OpNode, traverse deeper, else delete it
            if(curOp != nullptr){
                cout <<  " -- entering left node -- \n";
                treeClear(curOp->left);// clear left subtree

                cout <<  " -- entering right node -- \n";
                treeClear(curOp->right);// clear right subtree

                // if current node is an OpNode with no childern, delete it
                /*ShapeNode* lefNode = dynamic_cast<ShapeNode*>(curOp->left);                
                ShapeNode* rigNode = dynamic_cast<ShapeNode*>(curOp->right);
                if(lefNode == nullptr && rigNode == nullptr){
                    cout << " -- OpNode with no childern deleted -- \n";
                    delete curOp;
                }* //gives segmentation fault */
            }else if(curNode != nullptr){
                cout << " -- shape deleted -- \n";
                delete curNode;
            }
        }////////// ---- else{cout << " -- op deleted -- \n";delete curOp;}

    }catch(exception &e){cerr << " at Scene::treeClear - encountered problem when casting" << endl;}
}

void Scene::clear()
{
    geom.clear();
    vox.clear();

    // TO DO HERE, code to walk csg tree and deallocate nodes
    treeClear(csgroot);
    // will require dynamic casting of SceneNode pointers
}

bool Scene::bindGeometry(View * view, ShapeDrawData &sdd)
{
    if(voxactive)
    {
        return genVoxRender(view, sdd);
    }
    else
        return genVizRender(view, sdd);
}

void Scene::voxSetOp(SetOp op, VoxelVolume *leftarg, VoxelVolume *rightarg)
{
    // stub, needs completing
}

void Scene::voxWalk(SceneNode *root, VoxelVolume *voxels)
{
    // stub, needs completing
    // will require dynamic casting of SceneNode pointers
}

void Scene::voxelise(float voxlen)
{
    int xdim, ydim, zdim;

    // calculate voxel volume dimensions based on voxlen
    xdim = ceil(voldiag.i / voxlen)+2; // needs a 1 voxel border to ensure a closed mesh if shapes reach write up to the border
    ydim = ceil(voldiag.j / voxlen)+2;
    zdim = ceil(voldiag.k / voxlen)+2;
    voxsidelen = voxlen;
    voxactive = true;

    cgp::Vector voxdiag = cgp::Vector((float) xdim * voxlen, (float) ydim * voxlen, (float) zdim * voxlen);
    cgp::Point voxorigin = cgp::Point(-0.5f*voxdiag.i, -0.5f*voxdiag.j, -0.5f*voxdiag.k);
    vox.setDim(xdim, ydim, zdim);
    vox.setFrame(voxorigin, voxdiag);

    cerr << "Voxel volume dimensions = " << xdim << " x " << ydim << " x " << zdim << endl;

    // actual recursive depth-first walk of csg tree
    if(csgroot != NULL)
        voxWalk(csgroot, &vox);
}

void Scene::sampleScene()
{
    ShapeNode * sph = new ShapeNode();
    sph->shape = new Sphere(cgp::Point(0.0f, 0.0f, 0.0f), 4.0f);

    ShapeNode * cyl1 = new ShapeNode();
    cyl1->shape = new Cylinder(cgp::Point(-7.0f, -7.0f, 0.0f), cgp::Point(7.0f, 7.0f, 0.0f), 2.0f);

    ShapeNode * cyl2 = new ShapeNode();
    cyl2->shape = new Cylinder(cgp::Point(0.0f, -7.0f, 0.0f), cgp::Point(0.0f, 7.0f, 0.0f), 2.5f);

    OpNode * combine = new OpNode();
    combine->op = SetOp::UNION;
    combine->left = sph;
    combine->right = cyl1;

    OpNode * diff = new OpNode();
    diff->op = SetOp::DIFFERENCE;
    diff->left = combine;
    diff->right = cyl2;

    csgroot = diff;
}

void Scene::expensiveScene()
{
    ShapeNode * sph = new ShapeNode();
    sph->shape = new Sphere(cgp::Point(1.0f, -2.0f, -2.0f), 3.0f);

    ShapeNode * cyl = new ShapeNode();
    cyl->shape = new Cylinder(cgp::Point(-7.0f, -7.0f, 0.0f), cgp::Point(7.0f, 7.0f, 0.0f), 2.0f);

    ShapeNode * mesh = new ShapeNode();
    Mesh * bunny = new Mesh();
    bunny->readSTL("../meshes/bunny.stl");
    bunny->boxFit(10.0f);
    mesh->shape = bunny;

    OpNode * combine = new OpNode();
    combine->op = SetOp::UNION;
    combine->left = mesh;
    combine->right = cyl;

    OpNode * diff = new OpNode();
    diff->op = SetOp::DIFFERENCE;
    diff->left = combine;
    diff->right = mesh;

    csgroot = diff;
}