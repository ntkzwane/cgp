#if HAVE_CONFIG_H
# include <config.h>
#endif

#include <test/testutil.h>
#include "test_mesh.h"
#include <stdio.h>
#include <cstdint>
#include <sstream>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>

void TestMesh::setUp()
{
    mesh = new Mesh();
    scene = new Scene();
}

void TestMesh::tearDown()
{
    delete mesh;
    delete scene;
}

void TestMesh::testMeshing()
{
    // validity tests are still empty shells

    // mesh->readSTL("../meshes/bunny.stl");
    // CPPUNIT_ASSERT(mesh->basicValidity());
    // CPPUNIT_ASSERT(!mesh->manifoldValidity()); // bunny has known holes in the bottom
    // cerr << "BUNNY TEST PASSED" << endl;
    scene->expensiveScene();
}

//#if 0 /* Disabled since it crashes the whole test suite */
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION(TestMesh, TestSet::perBuild());
//#endif
